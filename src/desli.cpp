#include <VL53L0X.h>

// ============================
//  Definición de núcleos ESP32
// ============================
#define CORE_0 0   // núcleo base usado por algunas tareas del procesador
#define CORE_1 1   // núcleo para el usuario, tareas críticas

// ============================
//  Configuración del PWM
// ============================
#define FREQUENCY_PWM   1000  // frecuencia del PWM [Hz]
#define RESOLUTION_PWM  8     // bits del PWM
#define PIN_PWM         12    // pin del PWM
#define CH_PWM          0

// Factor de conversión de bits a voltios: 2^#bits - 1 / voltios
float voltsToPwm = (pow(2, RESOLUTION_PWM) - 1) / 24.0f;

// ============================
//  Sensor láser VL53L0X
// ============================
// Conectar GPIO 22 a SCL
// Conectar GPIO 21 a SDA
VL53L0X sensor;

bool setupSensor(void) {
    sensor.setTimeout(100);
    if (!sensor.init()) {
        Serial.println("No se detectó sensor");
        return false;
    }
    sensor.startContinuous();
    sensor.setMeasurementTimingBudget(20000);
    return true;
}

void setupPwm(void) {
    ledcSetup(CH_PWM, FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(PIN_PWM, CH_PWM);
    ledcWrite(CH_PWM, 128);  // 50% duty cycle inicial
}

void voltsToFan(float volts) {
    uint16_t pwm = volts * voltsToPwm;
    ledcWrite(CH_PWM, pwm);
}

// ============================
//  Parámetros de muestreo
// ============================
float Ts = 0.025f;  // período de muestreo [s] - 25ms

// Offset de calibración (ajustar experimentalmente)
float calibration = 42.0f / 2.0f;  // [cm]

// ============================
//  Parámetros de la planta
// ============================
float b = 10.089f;
float tau = 8.136835f;
float L = 0.637768f;  // retardo [s]

// Cálculo de muestras de retardo
int delay_samples = round(L / Ts);  // ~25 muestras

// ============================
//  Límites del actuador
// ============================
float umax = 24.0f;    // [V]
float umin = 10.0f;    // [V]
float u_eq_point = 16.25f;  // punto de operación [V]

// Compensación de zona muerta
float comp_dz = 0.05f * (umax - u_eq_point);  // ~0.39V

// ============================
//  Parámetros del controlador SMC
// ============================
// Parámetros conservadores apropiados para retardo grande
float lambda = 0.7f;   // pendiente de la superficie deslizante
float k_smc = 14.0f;   // ganancia del término de conmutación
float phi = 15.0f;     // ancho de la capa límite

// Filtro de la derivada
float alpha_filter = 0.4f;  // coeficiente del filtro pasa-bajos

// Estimador de bias (compensación de perturbaciones)
float gamma_var = 0.2f;         // tasa de adaptación
float bias_limit = 0.7f * (umax - u_eq_point);  // límite del bias
float kp_bias = 0.08f;      // ganancia proporcional adicional

// ============================
//  Variables del controlador
// ============================
float reference = 12.0f;  // referencia en cm
float y = 0.0f;           // posición medida [cm]
float dy = 0.0f;          // derivada de la posición [cm/s]
float dy_prev = 0.0f;     // derivada anterior
float dy_filt = 0.0f;     // derivada filtrada
float y_prev = 0.0f;      // posición anterior

float e = 0.0f;           // error
float de = 0.0f;          // derivada del error
float s = 0.0f;           // superficie deslizante

float u = 0.0f;           // señal de control total
float u_eq = 0.0f;        // control equivalente
float u_sw = 0.0f;        // término de conmutación
float usat = 0.0f;        // señal saturada

float bias_est = 0.0f;    // estimación del bias

// Buffer circular para el retardo
#define MAX_DELAY_SAMPLES 30
float u_buffer[MAX_DELAY_SAMPLES];
int buffer_index = 0;

// Referencia previa para calcular derivada
float r_prev = 12.0f;
float dr = 0.0f;

// ============================
//  Función saturación suave (tanh)
// ============================
float sat_smooth(float x, float phi_val) {
    return tanh(x / phi_val);
}

// ============================
//  Función de compensación de zona muerta
// ============================
float compDeadZone(float var, float dz) {
    if (var == 0.0f) {
        return 0.0f;
    }
    return var + (var > 0.0f ? dz : -dz);
}

// ============================
//  Declaración de tareas
// ============================
static void controlTask(void *pvParameters);
static void serialTask(void *pvParameters);

// ============================
//  Setup
// ============================
void setup() {
    // Inicializar comunicación I2C y sensor
    Wire.begin();
    while (!setupSensor()) {
        delay(1000);
    }

    setupPwm();
    delay(100);
    Serial.begin(115200);

    // Inicializar buffer de retardo con el punto de operación
    for (int i = 0; i < MAX_DELAY_SAMPLES; i++) {
        u_buffer[i] = u_eq_point;
    }

    // Tarea de control en el núcleo 1 (crítica)
    xTaskCreatePinnedToCore(
        controlTask,
        "SMC controller",
        8192,
        NULL,
        23,       // prioridad alta
        NULL,
        CORE_1
    );

    // Tarea para manejar comandos por Serial
    xTaskCreatePinnedToCore(
        serialTask,
        "serialTask",
        4096,
        NULL,
        1,
        NULL,
        CORE_0
    );
}

// ============================
//  Tarea de control SMC (crítica)
// ============================
static void controlTask(void *pvParameters) {
    const TickType_t taskInterval = (TickType_t)(1000.0f * Ts);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 1) Medir posición
        float medida_mm = (float)sensor.readRangeContinuousMillimeters();
        float medida_cm = medida_mm / 10.0f;
        y = calibration - medida_cm;

        // 2) Calcular derivada de la posición (método de diferencias)
        dy = (y - y_prev) / Ts;
        y_prev = y;

        // Filtro pasa-bajos para suavizar la derivada
        dy_filt = alpha_filter * dy_filt + (1.0f - alpha_filter) * dy;

        // 3) Calcular derivada de la referencia
        dr = (reference - r_prev) / Ts;
        r_prev = reference;

        // 4) Calcular error y su derivada
        e = reference - y;
        de = dr - dy_filt;

        // 5) Superficie deslizante (sin gain-scheduling)
        s = de + lambda * e;

        // 6) Control equivalente
        // u_eq = u0 + (r_dd + lambda*(dr - dy_filt) + (1/tau)*dy_filt) / (b/tau) + kp_bias*e + bias_est
        float r_dd = 0.0f;  // aceleración de la referencia (cero para escalones)
        float num = r_dd + lambda * (dr - dy_filt) + (1.0f / tau) * dy_filt;
        float den = b / tau;
        
        u_eq = u_eq_point + (num / den) + kp_bias * e + bias_est;

        // 7) Término de conmutación (switching)
        u_sw = k_smc * sat_smooth(s, phi);

        // 8) Señal de control total
        float u_raw = u_eq + u_sw;

        // 9) Compensación de zona muerta
        if (abs(u_raw - u_eq_point) > 0.5f) {
            u_raw = u_raw + ((u_raw > u_eq_point) ? comp_dz : -comp_dz);
        }

        // 10) Saturación
        u = constrain(u_raw, umin, umax);
        usat = u;

        // 11) Actualizar buffer circular para el retardo
        u_buffer[buffer_index] = u;
        buffer_index = (buffer_index + 1) % delay_samples;
        
        // Obtener la señal retardada
        float u_delayed = u_buffer[buffer_index];

        // 12) Enviar al ventilador
        voltsToFan(u_delayed);

        // 13) Actualización del estimador de bias (con anti-windup)
        bool saturado = (u >= umax - 1e-6f) || (u <= umin + 1e-6f);
        
        if (!saturado) {
            bias_est = bias_est + gamma_var * e * Ts;
        }
        
        // Limitar el bias
        bias_est = constrain(bias_est, -bias_limit, bias_limit);

        // 14) Logging para el Serial Plotter
        Serial.printf(">Posicion:%.2f, Referencia:%.2f, u:%.2f, error:%.2f\r\n",
                      y, reference, usat, e);

        // 15) Esperar hasta la siguiente muestra
        vTaskDelayUntil(&xLastWakeTime, taskInterval);
    }
}

// ============================
//  Tarea para comandos Serial
// ============================
static void serialTask(void *pvParameters) {
    (void)pvParameters;
    
    for (;;) {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();

            // Comando: "r 15.0" para cambiar referencia
            if (input.startsWith("r ")) {
                float new_ref = input.substring(2).toFloat();
                // Limitar referencia a valores seguros
                new_ref = constrain(new_ref, -23.0f, 25.0f);
                reference = new_ref;
                Serial.printf("Nueva referencia: %.2f cm\r\n", reference);
            }

            // Comando: "u0 16.5" para cambiar punto de operación
            else if (input.startsWith("u0 ")) {
                u_eq_point = input.substring(3).toFloat();
                Serial.printf("Nuevo u0: %.2f V\r\n", u_eq_point);
            }

            // Comando: "lambda 0.8" para ajustar lambda
            else if (input.startsWith("lambda ")) {
                lambda = input.substring(7).toFloat();
                Serial.printf("Nuevo lambda: %.2f\r\n", lambda);
            }

            // Comando: "k 15" para ajustar ganancia de conmutación
            else if (input.startsWith("k ")) {
                k_smc = input.substring(2).toFloat();
                Serial.printf("Nuevo k: %.2f\r\n", k_smc);
            }

            // Comando: "phi 20" para ajustar capa límite
            else if (input.startsWith("phi ")) {
                phi = input.substring(4).toFloat();
                Serial.printf("Nuevo phi: %.2f\r\n", phi);
            }

            // Comando: "reset" para resetear el estimador de bias
            else if (input.startsWith("reset")) {
                bias_est = 0.0f;
                dy_filt = 0.0f;
                Serial.println("Bias y filtros reseteados");
            }

            // Comando: "info" para ver parámetros actuales
            else if (input.startsWith("info")) {
                Serial.println("\n=== PARÁMETROS SMC ===");
                Serial.printf("lambda = %.2f\r\n", lambda);
                Serial.printf("k = %.2f\r\n", k_smc);
                Serial.printf("phi = %.2f\r\n", phi);
                Serial.printf("u0 = %.2f V\r\n", u_eq_point);
                Serial.printf("bias_est = %.3f V\r\n", bias_est);
                Serial.printf("Retardo = %.3f s (%d muestras)\r\n", L, delay_samples);
                Serial.println("=====================\n");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Poll cada 50 ms
    }
}

void loop() {
    vTaskDelete(NULL);
}
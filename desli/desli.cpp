#include <VL53L0X.h>

// ============================
//  Definición de núcleos ESP32
// ============================
#define CORE_0 0
#define CORE_1 1

// ============================
//  Configuración del PWM
// ============================
#define FREQUENCY_PWM   1000
#define RESOLUTION_PWM  8
#define PIN_PWM         12
#define CH_PWM          0

float voltsToPwm = (pow(2, RESOLUTION_PWM) - 1) / 24.0f;

// ============================
//  Sensor láser VL53L0X
// ============================
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
    ledcWrite(CH_PWM, 128);
}

void voltsToFan(float volts) {
    if (volts < 0) volts = 0;
    if (volts > 24) volts = 24;
    uint16_t pwm = volts * voltsToPwm;
    ledcWrite(CH_PWM, pwm);
}

// ============================
//  Parámetros de muestreo
// ============================
float Ts = 0.025f;

// Offset de calibración (recomendado igual que en PID/Hinf)
float calibration = 42.0f /2;  // [cm] 0 en el fondo aprox.

// ============================
//  Parámetros de la planta (para el modelo del SMC)
// ============================
float b    = 10.089f;
float tau  = 8.136835f;

// ============================
//  Límites del actuador
// ============================
float umax      = 24.0f;
float umin      = 10.0f;
float u_eq_point = 16.25f;

// Compensación de zona muerta
float comp_dz = 0.05f * (umax - u_eq_point);

// ============================
//  Parámetros del controlador SMC
// ============================
// Igual que en el MATLAB SMC simplificado
float lambda   = 0.35f;
float k_smc    = 7.0f;
float phi      = 25.0f;

// Filtro de la derivada
float alpha_filter = 0.4f;

// Estimador de bias
float gamma_var  = 0.2f;
float bias_limit = 0.7f * (umax - u_eq_point);
float kp_bias    = 0.08f;

// ============================
//  Variables del controlador
// ============================
float reference = -10.0f;  // [cm]
float y         = 0.0f;
float dy        = 0.0f;
float dy_filt   = 0.0f;
float y_prev    = 0.0f;

float e   = 0.0f;
float de  = 0.0f;
float s   = 0.0f;

float u       = 0.0f;
float u_eq    = 0.0f;
float u_sw    = 0.0f;
float usat    = 0.0f;

float bias_est = 0.0f;

float r_prev = 12.0f;
float dr     = 0.0f;

// ============================
//  Funciones auxiliares
// ============================
float sat_smooth(float x, float phi_val) {
    return tanh(x / phi_val);
}

float compDeadZone(float var, float dz) {
    if (var == 0.0f) return 0.0f;
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
    Wire.begin();
    while (!setupSensor()) {
        delay(1000);
    }

    setupPwm();
    delay(100);
    Serial.begin(115200);

    // Tarea de control en el núcleo 1
    xTaskCreatePinnedToCore(
        controlTask,
        "SMC controller",
        8192,
        NULL,
        23,
        NULL,
        CORE_1
    );

    // Tarea para comandos por Serial
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
//  Tarea de control SMC
// ============================
static void controlTask(void *pvParameters) {
    const TickType_t taskInterval = (TickType_t)(1000.0f * Ts);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 1) Medir posición
        float medida_mm = (float)sensor.readRangeContinuousMillimeters();
        float medida_cm = medida_mm / 10.0f;
        y = calibration - medida_cm;   // igual que en PID/Hinf

        // 2) Derivada por diferencias
        dy = (y - y_prev) / Ts;
        y_prev = y;

        // Filtro pasa-bajos
        dy_filt = alpha_filter * dy_filt + (1.0f - alpha_filter) * dy;

        // 3) Derivada de la referencia
        dr = (reference - r_prev) / Ts;
        r_prev = reference;

        // 4) Error y derivada del error
        e  = reference - y;
        de = dr - dy_filt;

        // 5) Superficie deslizante
        s = de + lambda * e;

        // 6) Control equivalente (como en simulateSMC_Simple)
        float r_dd = 0.0f;
        float num = r_dd + lambda * (dr - dy_filt) + (1.0f / tau) * dy_filt;
        float den = b / tau;

        u_eq = u_eq_point + (num / den) + kp_bias * e + bias_est;

        // 7) Término de conmutación
        u_sw = k_smc * sat_smooth(s, phi);

        // 8) Control total
        float u_raw = u_eq + u_sw;

        // 9) Compensación de zona muerta (si nos alejamos de u0)
        if (fabs(u_raw - u_eq_point) > 0.5f) {
            u_raw = u_raw + ((u_raw > u_eq_point) ? comp_dz : -comp_dz);
        }

        // 10) Saturación
        u    = constrain(u_raw, umin, umax);
        usat = u;

        // 11) Enviar directamente al ventilador (SIN retardo artificial)
        voltsToFan(u);

        // 12) Estimador de bias (anti-windup)
        bool saturado = (u >= umax - 1e-6f) || (u <= umin + 1e-6f);

        if (!saturado) {
            bias_est = bias_est + gamma_var * e * Ts;
        }
        bias_est = constrain(bias_est, -bias_limit, bias_limit);

        // 13) Log para plot
        Serial.printf(">Posicion:%.2f, Referencia:%.2f, u:%.2f, error:%.2f, s:%.2f\r\n",
                      y, reference, usat, e, s);

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

            if (input.startsWith("r ")) {
                float new_ref = input.substring(2).toFloat();
                new_ref = constrain(new_ref, -20.0f, 20.0f);
                reference = new_ref;
                Serial.printf("Nueva referencia: %.2f cm\r\n", reference);
            }
            else if (input.startsWith("u0 ")) {
                u_eq_point = input.substring(3).toFloat();
                Serial.printf("Nuevo u0: %.2f V\r\n", u_eq_point);
            }
            else if (input.startsWith("lambda ")) {
                lambda = input.substring(7).toFloat();
                Serial.printf("Nuevo lambda: %.2f\r\n", lambda);
            }
            else if (input.startsWith("k ")) {
                k_smc = input.substring(2).toFloat();
                Serial.printf("Nuevo k: %.2f\r\n", k_smc);
            }
            else if (input.startsWith("phi ")) {
                phi = input.substring(4).toFloat();
                Serial.printf("Nuevo phi: %.2f\r\n", phi);
            }
            else if (input.startsWith("reset")) {
                bias_est = 0.0f;
                dy_filt  = 0.0f;
                Serial.println("Bias y filtro de derivada reseteados");
            }
            else if (input.startsWith("info")) {
                Serial.println("\n=== PARÁMETROS SMC ===");
                Serial.printf("lambda = %.2f\r\n", lambda);
                Serial.printf("k      = %.2f\r\n", k_smc);
                Serial.printf("phi    = %.2f\r\n", phi);
                Serial.printf("u0     = %.2f V\r\n", u_eq_point);
                Serial.printf("bias_est = %.3f V\r\n", bias_est);
                Serial.printf("Ts     = %.3f s\r\n", Ts);
                Serial.println("=======================\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void loop() {
    vTaskDelete(NULL);
}
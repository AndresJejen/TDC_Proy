
#include <VL53L0X.h>
// definición de los nucleos en el esp32

#define CORE_0 0   // nucleo base usado por algunas tareas procesador
#define CORE_1 1   // nucleo para el usuario, poner aqui las tareas críticas


// Esta es la configuración del PWM
#define  FREQUENCY_PWM     1000 // frecuencia del pwa
#define  RESOLUTION_PWM    8  // bits del pwm
#define  PIN_PWM           12   // pin del pwm 1   
#define  CH_PWM     0
 
// factor de conversion de bits a voltios:  2^ #bits -1 / voltios
float  voltsToPwm =   (pow(2, RESOLUTION_PWM) - 1) / 24; 



//sensor laser
//conectar GPIO 22 SCL
//conectar GPIO 21 a SDA
VL53L0X sensor;


bool setupSensor(void){
    // we set the laser sensor
    sensor.setTimeout(100);  
    if(!sensor.init()){
        Serial.println("No se detecto sensor");
        return false;
        }
    sensor.startContinuous(); 
    sensor.setMeasurementTimingBudget(20000);
    return true;
}

void setupPwm(void){
    //ledcAttach(PIN_PWM,  FREQUENCY_PWM , RESOLUTION_PWM);
    // use las siguientes en PLATTFORMIO en vez de la anterior
    ledcSetup(CH_PWM, FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(PIN_PWM, CH_PWM);
    ledcWrite(CH_PWM, 128);  // 50% duty cycle
}

void voltsToFan(float volts){
    uint16_t pwm = volts * voltsToPwm;
    ledcWrite(CH_PWM , pwm); 
}


float movingAverage(float newValue) {
    const int filterSize = 20;
    static float filter_values[filterSize] = {0.0};  // Array to store the last 'FILTER_SIZE' values
    static int index = 0;  // Current index in the array
    static float sum = 0.0;  // Sum of the current values in the array
    static int count = 0;  // Number of values added to the filter

    // Subtract the oldest value from sum and replace it with the new value
    sum = sum - filter_values[index] + newValue;
    filter_values[index] = newValue;

    // Update the index, and wrap around if necessary
    index = (index + 1) % filterSize ;

    // Keep track of how many values have been added to the filter
    if (count < filterSize ) {
        count++;
    }
    // Return the average of the values in the filter
    return sum / count;
}

float compDeadZone(float var, float dz){
    // This function compensates the dead zone of DC motor
    if (var == 0){
        return 0;
    }
    else {
        //float sgnvar = abs(var)/var;
        return var + dz;
    }
}


// ============================
//  Parámetros de muestreo
// ============================

// tiempo de muestreo dejar en 25ms no cambiar
float h = 0.025f;   // [s]

// offset de calibracion para detectar la distancia en positivo con 0 en el
// piso del tubo (tu ajuste experimental)
float calibration = 42.0f / 2;   // en cm, porque divides mm/10

// ============================
//  Límites del actuador
// ============================

float umax = 24.0f;   // V
float umin = 5.0f;    // V  (puedes subir a 10.0f si quieres que nunca baje de 10V)
float deadZone = 0.0f;

// Punto de operación del ventilador (donde empieza a levitar)
float u0 = 16.0f;    // V (ajústalo luego si hace falta)


// ============================
//  Controlador H-infinito Kd
//  (discreto, Ts = 0.025 s)
// ============================

// Número de estados del controlador
const int NX = 5;

// Matriz Ad (5x5) – copiar desde MATLAB
float Ad[NX][NX] = {
    { 1.0000f,  0.0000f,  0.0000f,  0.0000f,   0.0000f },
    { 3.4298f,  0.1666f, -1.9432f, -7.7209f, -33.0668f },
    { 0.1715f,  0.0583f,  0.9028f, -0.3860f,  -1.6533f },
    { 0.0021f,  0.0007f,  0.0238f,  0.9952f,  -0.0207f },
    { 0.0000f,  0.0000f,  0.0001f,  0.0125f,   0.9998f }
};

// Matriz Bd (5x1)
float Bd[NX] = {
    0.0125f,
    0.0214f,
    0.0011f,
    0.0000f,
    0.0000f
};

// Matriz Cd (1x5)
float Cd[NX] = {
   38.5482f,
   -6.8880f,
  -19.9215f,
  -86.5510f,
 -371.6483f
};

// Escalar Dd
float Dd = 0.2409f;

// Estado interno del controlador
float xK[NX] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float k_hinf = 0.4f;

// ============================
//  Variables del lazo de control
// ============================

float reference = -23.0f;  // referencia en cm (mismo sistema que 'y')
float y;                 // posición medida (cm)
float u;                 // acción de control (útil para debug)
float e;                 // error
float usat;              // señal de control saturada enviada al ventilador

static void controlTask(void *pvParameters);
static void serialTask(void *pvParameters);


// ============================
//  Setup
// ============================

void setup() {
    // iniciamos el sensor
    Wire.begin();
    while (!setupSensor()) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    setupPwm();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.begin(115200);

    // Tarea de control en el núcleo 1
    xTaskCreatePinnedToCore(
        controlTask,
        "Hinf controller",
        8192,
        NULL,
        23,       // prioridad alta
        NULL,
        CORE_1
    );

    // Tarea para manejar comandos por Serial (cambiar referencia, etc.)
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
//  Tarea de control (crítica)
// ============================

static void controlTask(void *pvParameters) {

    // periodo en ticks (h [s] * 1000 ms/s)
    const TickType_t taskInterval = (TickType_t)(1000.0f * h);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {

        // 1) Medir posición
        // sensor.readRangeContinuousMillimeters() -> mm
        // /10 -> cm, por eso calibration está en cm
        float medida_mm = (float)sensor.readRangeContinuousMillimeters();
        float medida_cm = medida_mm / 10.0f;

        y = calibration - medida_cm;

        // 2) Calcular error
        e = reference - y;
        float e_max = 10.0f;  // por ejemplo, máximo 10 cm de error efectivo
        if (e > e_max)  e = e_max;
        if (e < -e_max) e = -e_max;
        // 3) Controlador H-infinito discreto
        // u_lin(k) = C*xK(k) + D*e(k)
        // 3) Controlador H-infinito discreto
        float u_lin = Dd * e;
        for (int i = 0; i < NX; ++i) {
            u_lin += Cd[i] * xK[i];
        }

        // Escalamos la salida del controlador (bajamos la “agresividad”)
        u_lin *= k_hinf;

        // 4) Añadir offset de operación
        float u_cmd = u0 + u_lin;
        u = u_cmd;   // guardar para debug si quieres

        // 5) Compensación de zona muerta (siempre puedes poner deadZone>0)
        u_cmd = compDeadZone(u_cmd, deadZone);

        // 6) Saturación al rango físico del ventilador
        usat = constrain(u_cmd, umin, umax);

        // 7) Mandar al ventilador
        voltsToFan(usat);

        // 8) Actualizar estado del controlador
        // xK(k+1) = Ad*xK(k) + Bd*e(k)
        float xNext[NX];
        for (int i = 0; i < NX; ++i) {
            float acc = Bd[i] * e;
            for (int j = 0; j < NX; ++j) {
                acc += Ad[i][j] * xK[j];
            }
            xNext[i] = acc;
        }
        for (int i = 0; i < NX; ++i) {
            xK[i] = xNext[i];
        }

        // 9) Logging para plotear
        Serial.printf(">Posicion:%0.2f, Referencia:%0.2f, u:%0.2f, error:%0.2f\r\n",
                      y, reference, usat, e);

        // 10) Esperar hasta la siguiente muestra
        vTaskDelayUntil(&xLastWakeTime, taskInterval);
    }
}


// ============================
//  Tarea para comandos por Serial
// ============================

static void serialTask(void *pvParameters) {
    (void) pvParameters;
    for (;;) {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();

            // cambiar referencia: comando "r 10.0" (por ejemplo)
            if (input.startsWith("r ")) {
                reference = input.substring(2).toFloat();
                Serial.printf("Nueva referencia: %0.2f\r\n", reference);
            }

            // (Opcional) cambiar u0 en tiempo real:
            if (input.startsWith("u0 ")) {
                u0 = input.substring(3).toFloat();
                Serial.printf("Nuevo u0: %0.2f V\r\n", u0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // poll every 50 ms
    }
}

void loop() {
    vTaskDelete(NULL);
}
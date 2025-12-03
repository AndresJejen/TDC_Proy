#include <definitions.h>

// tiempo de muestreo dejar en 25ms no cambiar
float h = 0.025;


// offset de calibracion para detectar la distancia en positivo con 0 en el 
//piso del tubo
float calibration = 44;


//limites del actuador
float umax = 24;
float umin = 10;
float deadZone = 0;

// PID antiguos (los dejé por compatibilidad, pero ya no se usan)
float kp = 0.09146488923656028;
float ki = 0.01302963989469797;
float kd = 0.1712176117412655;

// Parámetros de control (antiguos, se mantienen)
float Ts = h; // Tiempo de muestreo en segundos
float Taw = 0.9 * Ts; // Constante anti-windup
float N = 5.0; // Factor de ancho de banda del derivador
float beta = 0.0; // Factor de realimentación

// Definición de coeficientes del controlador (antiguos, se mantienen)
float bi = ki * Ts;
float ad = kd / (kd + N * Ts);
float bd = (kd * N) / (kd + N * Ts);
float br = Ts / Taw;

// Variables del controlador PID (no usadas por Hinf, pero se mantienen para compatibilidad)
float I = 0.0;
float D = 0.0;
float P = 0.0;
float prev_y = 0.0;

// variables del lazo de control
float reference = 0.0; 
float y; // angulo a controlar
float u; //accion de control
float e; // error
float usat; // señal de control saturada

// *** HINF discretizado (Tustin, Ts = 0.025) matrices Ad (7x7), Bd (7x1), C (1x7), D=0
// Las matrices fueron calculadas desde tu K continuo y discretizadas.
// NOTA: mantuve precisión en floats para uso en ESP32.
static float x_state[7] = {0,0,0,0,0,0,0};    // estados
static float x_new[7];

const float Ad[7][7] = {
    {  0.99999624f,  -0.0f,         -0.0f,         -0.0f,         -0.0f,         -0.0f,         -0.0f       },
    {  1.18451581f,   0.30422386f, -0.05172927f, -0.06909544f, -0.16239398f, -0.67953086f, -27.31252043f},
    {  0.79252975f,   0.77349545f,  0.55134947f, -0.23932834f, -0.25776885f, -0.47226824f, -18.27400666f},
    {  0.15850595f,   0.15469909f,  0.31026989f,  0.95213433f, -0.05155377f, -0.09445365f,  -3.65480133f},
    {  0.00792530f,   0.00773495f,  0.01551349f,  0.09760672f,  0.99742231f, -0.00472268f,  -0.18274007f},
    {  0.00009907f,   0.00009669f,  0.00019392f,  0.00122008f,  0.02496778f,  0.99994097f,  -0.00228426f},
    {  0.00001830f,  -0.00000874f, -0.00000057f, -0.00000001f,  0.00001718f,  0.00155312f,   0.99957762f}
};

const float Bd[7] = {
    0.01249998f,
    0.00740830f,
    0.00495671f,
    0.00099134f,
    0.00004957f,
    0.00000062f,
   -0.00000026f
};

const float Ck[7] = { 0.2846f, 3.749f, -0.01205f, -0.01611f, -0.03814f, -0.1583f, -6.564f };
const float Dk = 0.0f;

// Prototipos de tareas
static void controlTask(void *pvParameters);
static void serialTask(void *pvParameters);

void setup() {
    // iniciamos el sensor  
    Wire.begin();
    while(!setupSensor()){
      vTaskDelay(1000);
    }
    setupPwm();    
    vTaskDelay(100);
    Serial.begin(115200);
    
    // Asi definimos la tarea de control, de la máxima prioridad en el nucleo 1
    xTaskCreatePinnedToCore(
            controlTask, // nombre de la rutina
            "simple PID controller",
            8192,
            NULL,
            23, // prioridad de la tarea (0-24) , siendo 24 la prioridad más critica      
            NULL,
            CORE_1
    );  

    xTaskCreatePinnedToCore(
        serialTask,       // task function
        "serialTask",     // name
        4096,             // stack size
        NULL,             // params
        1,                // priority
        NULL,             // task handle
        CORE_0            // run on CORE_0
    );
}


/* *************************************************************************
*                     FUNCION CRITICA DE CONTROL
***************************************************************************/ 


static void controlTask(void *pvParameters) {

    // Aqui configuro cada cuanto se repite la tarea
    const TickType_t taskInterval = 1000*h;  // repetimos la tarea cada tiempo de muestreo en milisegundos

    // prototipo de una tarea repetitiva   
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount(); 

       // leemos el valor actual del ángulo
       y = calibration - ((float) sensor.readRangeContinuousMillimeters()/10 );
       
       e = reference - y;

       // -------------------------------
       // *** CAMBIO HINF: actualizacion de estados y calculo de u ***
       // x_{k+1} = Ad * x_k + Bd * y_k
       for (int i = 0; i < 7; ++i) {
           float acc = 0.0f;
           for (int j = 0; j < 7; ++j) {
               acc += Ad[i][j] * x_state[j];
           }
           acc += Bd[i] * y;
           x_new[i] = acc;
       }
       // copiar estados
       for (int i = 0; i < 7; ++i) x_state[i] = x_new[i];

       // salida u = C * x + D * y
       float u_calc = 0.0f;
       for (int i = 0; i < 7; ++i) u_calc += Ck[i] * x_state[i];
       u_calc += Dk * y; // Dk = 0 en este diseño

       // aplicar compensacion de zona muerta y saturacion (igual que antes)
       usat = compDeadZone(u_calc, deadZone);
       usat = constrain(usat, umin, umax);
       voltsToFan(usat);
       // ------------------------------------------------------------

       // mantenemos impresion igual al original para compatibilidad con tu graficacion
       Serial.printf(">Posicion:%0.2f, Referencia:%0.2f, usat:%0.2f\r\n", y, reference, usat);
      
       // la tarea es crítica entonces esperamos exactamente taskInterval ms antes de activarla nuevamente
       vTaskDelayUntil(&xLastWakeTime, taskInterval);
    }

}

// Serial handling moved to its own FreeRTOS task
static void serialTask(void *pvParameters) {
    (void) pvParameters;
    for (;;) {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input.startsWith("r ")) {
                reference = input.substring(2).toFloat();
                Serial.printf("Nueva referencia: %0.2f\r\n", reference);
            }

            if (input.startsWith("p ")) {
                kp = input.substring(2).toFloat();
                Serial.printf("Nuevo kp: %0.2f\r\n", kp);
            }

            if (input.startsWith("i ")) {
                ki = input.substring(2).toFloat();
                Serial.printf("Nuevo ki: %0.2f\r\n", ki);
            }

            if (input.startsWith("d ")) {
                kd = input.substring(2).toFloat();
                Serial.printf("Nuevo kd: %0.2f\r\n", kd);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // poll every 50 ms
    }
}

void loop() {
    vTaskDelete(NULL);
}
#include <definitions.h>

// tiempo de muestreo dejar en 25ms no cambiar
float h = 0.025;


// offset de calibracion para detectar la distancia en positivo con 0 en el 
//piso del tubo
float calibration = 42;


//limites del actuador
float umax = 24;
float umin = 10;
float deadZone = 0;

float kp = 0.09146488923656028;
float ki = 0.01302963989469797;
float kd = 0.1712176117412655;

// Parámetros de control
float Ts = h; // Tiempo de muestreo en segundos
float Taw = 0.9 * Ts; // Constante anti-windup
float N = 5.0; // Factor de ancho de banda del derivador
float beta = 0.0; // Factor de realimentación

// Definición de coeficientes del controlador
float bi = ki * Ts;
float ad = kd / (kd + N * Ts);
float bd = (kd * N) / (kd + N * Ts);
float br = Ts / Taw;

// Variables del controlador
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

    const TickType_t taskInterval = 1000*h;  // repetimos la tarea cada tiempo de muestreo en milisegundos = 1000*0.01= 100ms
    


    // prototipo de una tarea repetitiva   
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount(); 

       // leemos el valor actual del ángulo
       y = calibration - ((float) sensor.readRangeContinuousMillimeters()/10 );
       
       e = reference - y;

        // Acción proporcional
        P = kp * (beta*reference-y);
        
        // Acción derivativa con filtro
        D = ad * D - bd * (y - prev_y);
        
        // Acción integral con anti-windup
        u = P + I + D;
        usat = compDeadZone(u, deadZone);
        usat = constrain(usat, umin, umax);
        voltsToFan(usat);

        I = I + bi * e + br * (usat - u);
        prev_y = y;
        // realizamos la compensacion de zona muerta del motor u = u + deadzone * sign(u), si |u|>0
      


       // ahora imprimimos para plotear al funcionamiento del controlador 
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
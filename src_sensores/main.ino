#include "definitions.h"
#include <Wire.h>

// tiempo de muestreo
float h = 0.05;
float distancia;
float voltaje;
bool  senDetectado = false;


void setup() {
    // iniciamos el sensor  
    Wire.begin();
    senDetectado = setupSensor();
    setupPwm();
    
    vTaskDelay(100);
    
    //Asi definimos la tarea de control, de la máxima prioridad en el nucleo 1
    xTaskCreatePinnedToCore(
            controlTask, // nombre de la rutina
            "general controller task",
            8192,
            NULL,
            23, // prioridad de la tarea (0-24) , siendo 24 la prioridad más critica      
            NULL,
            CORE_1
    );
}


/* *************************************************************************
*                    SENSOR LASER y VOLTAJE
***************************************************************************/ 


static void controlTask(void *pvParameters) {

    // Aqui configuro cada cuanto se repite la tarea
    const TickType_t taskInterval = 1000*h;  // repetimos la tarea cada tiempo de muestreo en milisegundos = 1000*0.025= 25ms
    

    // prototipo de una tarea repetitiva   
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount(); 
       
     
       // detección de la posicion en centimetros   
       distancia = (float) sensor.readRangeContinuousMillimeters()/10 -7 ;

       // envio de voltaje
       voltaje = 13;
       voltsToFan(voltaje);

       // ahora imprimimos para plotear al funcionamiento del sensor 
       if(senDetectado){
         printf("distancia:%0.2f ", distancia);  
       }
       // la tarea es crítica entonces esperamos exactamente taskInterval ms antes de activarla nuevamente
       vTaskDelayUntil(&xLastWakeTime, taskInterval);
    }

}





void loop() {
    vTaskDelete(NULL);
}
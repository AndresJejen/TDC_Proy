#include <VL53L0X.h>

// definición de los nucleos en el esp32

#define CORE_0 0   // nucleo base usado por algunas tareas procesador
#define CORE_1 1   // nucleo para el usuario, poner aqui las tareas críticas

// Esta es la configuración del PWM
#define  FREQUENCY_PWM     10000 // frecuencia del pwa
#define  RESOLUTION_PWM    8  // bits del pwm
#define  PIN_PWM           12   // pin del pwm 1   
#define  CH_PWM     0

float  voltsToPwm =   (pow(2, RESOLUTION_PWM) - 1) / 24.0; 


//sensor laser
//conectar GPIO 22 SCL
//conectar GPIO 21 a SDA
VL53L0X sensor;


bool setupSensor(void){
    // we set the laser sensor
    sensor.setTimeout(100);  
    if(!sensor.init()){
        printf("No se detecto sensor\n");
        return false;
        }
    sensor.startContinuous(); 
    return true;
}

void setupPwm(void){
    //ledcAttachPin(PIN_PWM,  FREQUENCY_PWM , RESOLUTION_PWM);
    // use las siguientes en PLATTFORMIO en vez de la anterior
    ledcSetup(CH_PWM, FREQUENCY_PWM, RESOLUTION_PWM);
    ledcAttachPin(PIN_PWM, CH_PWM);
    ledcWrite(CH_PWM, 128);  // 50% duty cycle
}

void voltsToFan(float volts){
    uint16_t pwm = volts * voltsToPwm;
    printf("pwm: %d\n", pwm);  
    ledcWrite(CH_PWM ,pwm); 
}




#include "system_init.h"
#include "scheduler.h"

// Subsystems (empty for now — you will add them gradually)
//#include "motors.h"
//#include "adc.h"
#include "temp_sensor.h"
//#include "ina219_sensor.h"
//#include "ultrasonic_sensor.h"
//#include "wifi_sys.h"
//#include "espnow_sys.h"
//#include "mqtt_sys.h"
//#include "ui_led.h"
//#include "ui_buttons.h"

void app_main(void)
{
    // --- System-level initialization ---
    system_init();

    // --- Subsystem instances ---
    //static motor_system_t       motors;
    //static adc_system_t         adc;
    static temp_sensor_system_t temp;
    //static ina219_system_t      ina;
    //static ultrasonic_system_t  ultra;
    //static mqtt_system_t        mqtt;

    // --- Initialize subsystems (you will add these one by one) ---
    //motor_system_init(&motors);
    //adc_system_init(&adc);
    temp_sensor_system_init(&temp);
    //ina219_system_init(&ina);
    //ultrasonic_system_init(&ultra);

    wifi_system_init();                 // stateless
    espnow_system_init(&adc, &motors);  // passes references
    //mqtt_system_init(&mqtt);

    //ui_led_init();
    //ui_buttons_init();

    // --- Scheduler wiring ---
    //static scheduler_t sched = {
    //    .motors = &motors,
    //    .adc    = &adc,
    //    .temp   = &temp,
    //    .ina    = &ina,
    //    .ultra  = &ultra,
    //    .mqtt   = &mqtt
    //};

    scheduler_init(&sched);
    scheduler_start(&sched);
}

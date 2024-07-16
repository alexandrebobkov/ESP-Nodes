const char* WIFI_SSID = "IoT_bots";
const char* WIFI_PASSWORD = "208208208";

/*
    ESP32-C3 Super Mini
    SDA     -> GPIO6
    SCL     -> GPIO7

    SCK     -> GPIO8
    MISO    -> GPIO9
    MOSI    -> GPIO10

    RX      -> GPIO20
    TX      -> GPIO21

    ADC1    -> GPIO1    Used for light sensor
    ADC2    -> GPIO0

    A3      -> GPIO5
    A2      -> GPIO4
    A1      -> GPIO3
    A0      -> GPIO2
    
*/
#define SDA_PIN         (6)
#define SCL_PIN         (7)
#define LED_PIN         (8)
#define SYS_LED_PIN     (8)
#define ADC1            (1)
#define LIGHTS_PIN      (10)

int light_sensor_reading = 0;
float lighting_percentage = 0.00;


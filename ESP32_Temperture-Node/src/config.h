// Uncomment modules as required
//#define RTC
//#define MICRO_SD
//#define BMP280    // Adafruit BMP280; temp & pressure (rectangular shape miniature IC)
#define BME280      // Generic I2C BME280; temp, pressure & humidity (square shape miniature IC)
/*
    BME-280 
    SCL -> GPIO 22
    SDA -> GPIO 21
    VIN -> 5V
*/
//#define AWSIoT

#define MQTT_SSL
//#define HOTSPOT
//#define MQTT


/*
#########################################################################

                                MQTT

#########################################################################
*/
// MQTT Default Channels
#define MQTT_IOT_CHANNEL_TEMPERATURE        "nodes/outdoors/foxie2/sensors/temperature"
#define MQTT_IOT_CHANNEL_PRESSURE           "nodes/outdoors/foxie2/sensors/pressure"
#define MQTT_IOT_CHANNEL_HUMIDITY           "nodes/outdoors/foxie2/sensors/humidity"
// MQTT Channel specific for each node
// [NODE]/sensors/sensor
// IoT ID
#define IoT_ID                              "node1"
#define MQTT_NODE_TEMPERATURE               "/sensors/temperature"
#define MQTT_NODE_PRESSURE                  "/sensors/pressure"
#define MQTT_NODE_HUMIDITY                  "/sensors/humidity"
#define MQTT_NODE_PULSE                     "/pulse"
#define MQTT_NODE_SWITCH_1                  "/sw1"
#define MQTT_NODE_SWITCH_2                  "/sw2"
#define MQTT_NODE_OUTPUT_PWM_1              "/pwm1"
// Standardize switches channels
#define MQTT_IOT_CHANNEL_OUTPUT_PULSE       "node1/output/pulse"
#define MQTT_IOT_CHANNEL_OUTPUT_SWITCH_1    "node1/output/sw1"
#define MQTT_IOT_CHANNEL_OUTPUT_SWITCH_2    "node1/sw2"
#define MQTT_IOT_CHANNEL_OUTPUT_PWM_1       "node1/pwm1"
#define MQTT_IOT_CHANNEL_0                  "test_topic"

// Uncomment corresponding board
//#define devkit_30pin_001
//#define devkit_36pin_001
#define devkit_v4_38pin_ext_antenna

#ifdef devkit_v4_38pin_ext_antenna
/*
#### ESP32 DEVKIT V4 DIY MALL             ####
#### 38 PINS External Antenna             ####
#### DEVELOPMENT BOARD SUPPORTED PIN OUTS ####
----------------------------------------------
GPIO    | Physical  |   Description
Pin     |           |
----------------------------------------------
              2         Built-in LED
D15           3
D2            4     =>  Built-in LED     
D4            5
D5            8
D18           9         
D19           10                
D21           11        
D22           14        
D23           15        
D34           19        Input only        
D35           20        Input only        
D32           21*   =>  SCL I2C        
D33           22*   =>  SDA I2C
D25           23*   =>  Assigned to DAC
D26           24*   =>  Assigned to DAC
D27           25
D14           26*   =>  Assigned to Switch 1
D12           27*   =>  Assigned to Switch 2
D13           28    =>  Assigned to read RPM
----------------------------------------------
*/
#define PING_PIN  33  // GPIO 33 pin # of audio ping
#define LED_PIN   32  // GPIO 32 pin # for system status LED
#define SWITCH_1  14  // GPIO 14
#define SWITCH_2  12  // GPIO 12
#define DAC_CH1   25  // GPIO 25
#define DAC_CH2   26  // GPIO 26
#define FAN_RPM   13  // GPIO 13
//uint8_t pins[] = {2,4,5,12,13,14,15,18,19,21,22,23,25,26,27,32,33}; // 20 GPIO pins
#endif

#ifdef devkit_30pin_001
/*
#### ESP32 DEVKIT V1.1 DIY MALL           ####
#### 30 PINS                              ####
#### DEVELOPMENT BOARD SUPPORTED PIN OUTS ####
----------------------------------------------
GPIO    | Physical  |   Description
Pin     |           |
----------------------------------------------
              2         Built-in LED
D15           3
D2            4     =>  Built-in LED     
D4            5
D5            8
D18           9         
D19           10                
D21           11        
D22           14        
D23           15        
D34           19        Input only        
D35           20        Input only        
D32           21*        
D33           22*
D25           23*   =>  Assigned to DAC
D26           24*   =>  Assigned to DAC
D27           25
D14           26*   =>  Assigned to Switch 1
D12           27*   =>  Assigned to Switch 2
D13           28    =>  Assigned to read RPM
----------------------------------------------
*/
#define PING_PIN  33  // GPIO 33 pin # of audio ping
#define LED_PIN   32           
#define SWITCH_1  14  // GPIO 14
#define SWITCH_2  12  // GPIO 12
#define DAC_CH1   25  // GPIO 25
#define DAC_CH2   26  // GPIO 26
#define FAN_RPM   13  // GPIO 13
//uint8_t pins[] = {2,4,5,12,13,14,15,18,19,21,22,23,25,26,27,32,33}; // 20 GPIO pins
#endif

/*
#### ESP32 DEVKIT V1.1 DIY MALL           ####
#### 36 PINS                              ####
#### DEVELOPMENT BOARD SUPPORTED PIN OUTS ####
----------------------------------------------
GPIO    | Physical  |   Description
Pin     |           |
----------------------------------------------
              2         Built-in LED
D0            5
D15           6
D2            7         Same as built-in LED
D4            8
D5            11
D18           12        
D19           13        
D21           14        
D22           17        
D23           18        
D34           22        
D35           23        
D32           24        
D33           25
D25           26
D26           27
D27           28
D17           29
D14           30
D12           31
D13           32
----------------------------------------------
*/
#ifdef devkit_36pin_001
#define PING_PIN 33           // D33 pin # of audio ping
#define LED_PIN   2            // pin # of LED controlled by light sensor
//uint8_t pins[] = {};
#endif
/*
##############################################
*/

//#define LIGHT_SENSOR_PIN 34   // analog in pin # for a light sensor
//#define LED_PIN 32            // pin # of LED controlled by light sensor
#define ANALOG_THRESHOLD 1800 // threshhold for analog input when logical 0 should become logical 1
// RGB LED
#define RGB_R_PIN 11  // D14
#define RGB_B_PIN 13  // D13
#define RGB_G_PIN 12  // D12
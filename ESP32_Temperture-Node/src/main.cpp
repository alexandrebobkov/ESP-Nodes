/*
  Written for ESP32 development board DEVKIT v1 (Espressive ESP32-WROOM-32)
    
  Node sending temperature and atmosphere pressure readings via MQTT.
  
  by: Alexander Bobkov
  Created: December 17, 2023
  Updated:
  
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

#include "secrets.h"
#include "config.h"

struct {
  float humidity = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
} sensors_values;

// Sensors modules specified in config.h file.
// BME280
#ifdef BME280
// WaveShare BME280
Adafruit_BME280 bme;
#define SEALEVELPRESSURE_HPA (1013.25)
#endif
// BMP280
#ifdef BMP280
#define BMP_SCK   (18)
#define BMP_MISO  (19)
#define BMP_MOSI  (23)
#define BMP_CS    (5)
//Adafruit_BMP280 bmp(BMP_CS);
#endif

// Mosquitto
#ifdef MQTT
WiFiClient espClient;
PubSubClient connection(espClient);
Mosquitto mosquitto = Mosquitto();
#endif
#ifdef MQTT_SSL
WiFiClientSecure espClientSSL = WiFiClientSecure();
PubSubClient connection(espClientSSL); //mosquitto_ssl
//Mosquitto mosquitto = Mosquitto();
#endif

void mosquito_callback (char* topic, byte* message, unsigned int length)
{
  //mosquitto.mosquito_callback(topic, message, length);

  // Display topic and message received.
  Serial.print("\nMessage arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  String messageTemp;                               // variable to temporary store message received

  // Convert message received in bytes into String
  for (int i=0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  Serial.println(messageTemp);

  // Identify message received
  if (strstr(topic, "esp32/sw1")) {
    Serial.print("===== switch 1 ");
    if (messageTemp == (String)"1") {
      Serial.println("on =====");
      digitalWrite(SWITCH_1, HIGH);
    }
    if (messageTemp == (String)"0") {
      Serial.println("off =====");
      digitalWrite(SWITCH_1, LOW);
    }
  }
  if (strstr(topic, "esp32/sw2")) {
    Serial.println("switch 2");
    if (messageTemp == (String)"on") {
      Serial.println("on =====");
      digitalWrite(SWITCH_2, HIGH);
    }
    if (messageTemp == (String)"off") {
      Serial.println("off =====");
      digitalWrite(SWITCH_2, LOW);
    }
  }

  if (strstr(topic, MQTT_IOT_CHANNEL_OUTPUT_SWITCH_1)) {
  //if (strcmp((char *)topic, MQTT_IOT_CHANNEL_OUTPUT_SWITCH_1)) {
    Serial.println(MQTT_IOT_CHANNEL_OUTPUT_SWITCH_1);
    if (messageTemp == (String)"on") {
    //if (strcmp(messageTemp, "on")) {
      Serial.println("Switch 1 ON\n");
      digitalWrite(12, HIGH);
    }
    if (messageTemp == (String)"off") {
    //if (strcmp(messageTemp, "off")) {
      Serial.println("Switch 1 OFF\n");
      digitalWrite(12, LOW);
    }
  }
  if (strstr(topic, MQTT_IOT_CHANNEL_OUTPUT_SWITCH_2)) {
    Serial.println("switch 2");
    if (messageTemp == (String)"on") {
      Serial.println("on =====");
      digitalWrite(12, HIGH);
    }
    if (messageTemp == (String)"off") {
      Serial.println("off =====");
      digitalWrite(12, LOW);
    }
  }
  if (strstr(topic, MQTT_IOT_CHANNEL_OUTPUT_PWM_1)) {
    Serial.println(MQTT_IOT_CHANNEL_OUTPUT_PWM_1);
    if (messageTemp == (String)"on") {
      Serial.println("on =====");
      //digitalWrite(DAC_CH1, HIGH);
      dacWrite(DAC1, 255);
    }
    /*else if (messageTemp == (String)"off") {
      Serial.println("off =====");
      //digitalWrite(DAC_CH1, LOW);
      dacWrite(DAC1, 0);
    }
    else {
      int pwm1 = (int)message;
      Serial.print("PWM-1: ");
      Serial.println(pwm1);
      //digitalWrite(DAC_CH1, pwm1);
      dacWrite(DAC1, 255);
    }*/
    else {
      int pwm1 = messageTemp.toInt();
      Serial.print("PWM-1: ");
      Serial.println(pwm1);
      dacWrite(DAC1, pwm1);
    }
  }
}

// Connect to Mosquito MQTT server.
void mosquitto_connect ()
{
  /*
  Statuses for connected():
    0   => Connected
    -2
    -4
  */
  while (!connection.connected()) {
    Serial.print("MQTT connection state: ");
    Serial.println(connection.state());
    Serial.print("Connecting to Mosquitto at IP: ");
    Serial.print(mqtt_server);
    #ifdef MQTT // MOSQUITTO MQTT port 1883
    Serial.println(":1883");
    connection.setServer(mqtt_server, 1883);
    if(connection.connect("ESP32")) {
      Serial.println("Mosquitto Connected!");
      connection.setCallback(mosquito_callback);
    }
    Serial.print("Mosquitto state: ");
    Serial.println(connection.state());
    #endif
    #ifdef MQTT_SSL // MOSQUITTO MQTT port 8883
    Serial.println(":8883");
    connection.setServer(mqtt_server, 8883);
    espClientSSL.setCACert(NODE_CERT_CA);
    espClientSSL.setCertificate(NODE_CERT_CRT);
    espClientSSL.setPrivateKey(NODE_CERT_PRIVATE);
    if(connection.connect("ESP32")) {
      Serial.println("Mosquitto Connected!");
      connection.setCallback(mosquito_callback);
      connection.subscribe("esp32/sw1");
      connection.subscribe("esp32/sw2");
      connection.subscribe(MQTT_IOT_CHANNEL_OUTPUT_SWITCH_1);
      connection.subscribe(MQTT_IOT_CHANNEL_OUTPUT_SWITCH_2);
      connection.subscribe(MQTT_IOT_CHANNEL_OUTPUT_PWM_1);
    }
    Serial.print("Mosquitto state: ");
    Serial.println(connection.state());
    #endif
    delay(2000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  sensors_values.humidity = 0.0;
  sensors_values.pressure = 0.0;
  sensors_values.temperature = 0.0;

  // Initialize GPIO
  #ifdef devkit_36pin_001
  #endif
  #ifdef devkit_30pin_001
  pinMode(LED_PIN, OUTPUT);
  pinMode(PING_PIN, OUTPUT);
  pinMode(SWITCH_1, OUTPUT);
  pinMode(SWITCH_2, OUTPUT);

  pinMode(FAN_RPM, INPUT);
  digitalWrite(FAN_RPM, HIGH);
  //attachInterrupt(digitalPinToInterrupt(FAN_RPM), rpm_fan, FALLING);
  //pinMode(DAC_CH1, OUTPUT);
  // Active level is LOW
  digitalWrite(SWITCH_1, LOW);
  digitalWrite(SWITCH_2, LOW);
  //digitalWrite(DAC_CH1, LOW);
  dacWrite(DAC1, 0);
  #endif  

  Serial.println("setup");  
  Serial.println("setup done");

  // WaveShare BME280
  #ifdef BME280
  unsigned status = bme.begin(); 
  if (!status) {
    Serial.println("Could not find a valid BME/BMP280 sensor, check wiring!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID());//,16);
    Serial.print("   ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("   ID of 0x60 represents a BME 280.\n");
    Serial.print("   ID of 0x61 represents a BME 680.\n");
    while (1);
  }
  else {
    humidity = bme.readHumidity();
    pressure = bme.readPressure()  / 100.0F;
  }
  #endif

  // BMP280
  #ifdef BMP280  
  unsigned status_bmp280;
  status_bmp280 = bmp.begin();
  if (!status_bmp280) {
    Serial.println("Could not find BMP280");
    Serial.println(bmp.sensorID(),16);
    while (1);
  }
  else {
    Serial.println(bmp.sensorID(),16);
  }
  #endif

  // Initialize RTC module, if defined
  #ifdef RTC
  rtc.begin();  
  if (! rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  temp = rtc.getTemperature();
  #endif  
  // Uncomment when compiling for the first time
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  /*if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date &amp; time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date &amp; time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }*/

  String hostname = "ESP32LF";
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  
  Serial.println("Connecting to Wi-Fi");
  
  // Connect to wifi.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("#");
  }
  Serial.print("\nCONNECTED\nIP: ");  
  Serial.println(WiFi.localIP());
  
  Serial.print("Connecting to Mosquitto at IP: ");
  Serial.print(mqtt_server);
  #ifdef MQTT // MOSQUITTO MQTT port 1883
  Serial.println(":1883");
  connection.setServer(mqtt_server, 1883);
  if(connection.connect("ESP32")) {
    Serial.println("Mosquitto Connected!");
    connection.setCallback(mosquito_callback);
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    Serial.print("Mosquitto state: ");
    digitalWrite(LED_PIN, LOW);
  }
  Serial.println(connection.state());
  #endif
  #ifdef MQTT_SSL // MOSQUITTO MQTT port 8883
  Serial.println(":8883");
  connection.setServer(mqtt_server, 8883);
  espClientSSL.setCACert(NODE_CERT_CA);
  espClientSSL.setCertificate(NODE_CERT_CRT);
  espClientSSL.setPrivateKey(NODE_CERT_PRIVATE);
  //connection.setCallback(mosquito_callback);
  if(connection.connect("esp32")) {
    Serial.println("Mosquitto Connected!");
    connection.subscribe("esp32/sw1");
    connection.subscribe("esp32/sw2");
    connection.subscribe(MQTT_IOT_CHANNEL_OUTPUT_SWITCH_2);
    connection.subscribe(MQTT_IOT_CHANNEL_OUTPUT_PWM_1);
    connection.setCallback(mosquito_callback);
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    Serial.print("Mosquitto state: ");
    digitalWrite(LED_PIN, LOW);
  }
  Serial.println(connection.state());
  #endif  
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
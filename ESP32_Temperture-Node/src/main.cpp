/*
  Written for ESP32 development board DEVKIT v1 (Espressive ESP32-WROOM-32)
    
  Node sending temperature and atmosphere pressure readings via MQTT.
  
  by: Alexander Bobkov
  Created: December 17, 2023
  Updated:
  
*/

#include <Arduino.h>

struct {
  float humidity = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
} sensors_values;

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
Adafruit_BMP280 bmp(BMP_CS);
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
  
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
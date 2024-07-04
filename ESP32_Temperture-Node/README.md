# ESP32 MQTT SSL Temperature Node

<!-- BMP280 comes in 3.3V and 5V versions. 5V version is I2C and has 4 terminals; 3.3V version is IPS and has 6 terminals AHT10 Arduino sensor -->

## MQTT Mosquito Broker
<p>Mosquitto broker can be easily deployed using Docker compose file shown below. In the example below, MQTT broker listens to the ports 1883 (unecrypted) and 8883 (encrypted SSL). </p>

> [!NOTE]
> Compose file below declares two volumes (config and data) to persistantly store Mosquitto configuration and data.

```text
version: "3.8"
services:
  mosquitto-esp32:
    image: eclipse-mosquitto:latest
    volumes:
      - /srv/dev-disk-by-label/docker/volumes/mosquitto/config:/mosquitto/config
      - /srv/dev-disk-by-label/docker/volumes/mosquitto/data:/mosquitto/data
    networks:
      - IoT
    ports:
      - 1883:1883
      - 8883:8883
      - 9001:9001
    restart: unless-stopped
networks:
  IoT:
    external: true
```

<p><i>mosquitto.conf</i></p>

> [!NOTE]
> If you are using IP addresses, then issue certificates and keys to the corresponding IP address of MQTT brocker

``` text
## List ports listen to
listener 1883
listener 8883

cafile /mosquitto/config/certs/esp32_ca.crt
certfile /mosquitto/config/certs/esp32.crt
keyfile /mosquitto/config/certs/esp32.key

allow_anonymous true
persistence true
persistence_location /srv/dev-disk-by-label/docker/columes/mosquitto/data/
```

## ESP32 MQTT Client

<p>The two lines of code shown below are the most crucial as they are responsible for connecting ESP32 to the MQTT broker.</p>

> [!NOTE]
> A set of SSL certificates and key used by ESP32 MQTT client must correspond to ones used by MQTT brocker. Otherwise, secure connection won't be established. 

```C
espClientSSL.setCACert(NODE_CERT_CA);
espClientSSL.setCertificate(NODE_CERT_CRT);
espClientSSL.setPrivateKey(NODE_CERT_PRIVATE);
connection.setServer(mqtt_server, 8883);    // mqtt_server -> 192.168.50.16
```

<p><i>secrets.h</i></p>

> [!NOTE]
> Create file called secrets.h to store configuration information about Wi-Fi, and encryption keys, but exclude it from being pushed to GitHub

```text
const char* WIFI_SSID = "IoT_bots";
const char* WIFI_PASSWORD = "212212212";
const char* mqtt_server = "192.168.50.16";

// MQTT Broker Root CA
static const char NODE_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
< Cut&Paste content of CA certificate over here >
-----END CERTIFICATE-----
)EOF";

// MQTT Client Certificate
static const char NODE_CERT_CRT [] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
< Cut&Paste content of client certificate over here >
-----END CERTIFICATE-----
)EOF";

// MQTT Client Key
static const char NODE_CERT_PRIVATE [] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
< Cut&Paste content of client key over here >
-----END CERTIFICATE-----
)EOF";

```
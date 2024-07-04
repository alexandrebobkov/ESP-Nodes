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
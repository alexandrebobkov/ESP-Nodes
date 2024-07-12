from machine import Pin, I2C
#import machine
import time
import bosch_bme280


## MAIN LOOP SECTION

led = Pin(15, Pin.OUT)

i2c = I2C(1, scl=Pin(9), sda=Pin(8), freq=10000)

while True:
    bme = bosch_bme280.BME280(i2c=i2c)
    temp = bme.temperature
    hum = bme.humidity
    pres = bme.pressure
    
    print('\n')
    print('Temperature: ', temp)
    print('Humidity: ', hum)
    print('Pressure: ', pres)
    
    time.sleep(0.25)
    led.value(1)
    time.sleep(0.25)
    led.value(0)
    time.sleep(0.25)
    led.value(1)
    time.sleep(0.25)
    led.value(0)
    time.sleep(1)



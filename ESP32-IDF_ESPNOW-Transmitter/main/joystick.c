#include "sensors_data.h"

static sensors_data_t buffer;

int convert_axis_to_pwm(int axis_value) {
    // Convert the joystick axis value to a PWM value
    // Assuming axis_value is in the range of 0-4095 for a 12-bit ADC
    // and we want to map it to a PWM range of 0-255
    return (axis_value * 255) / 4095;
}

void sendRawData(void) {
    
    buffer.crc = 0;
    buffer.x_axis = 240;
    buffer.y_axis = 256;
    buffer.nav_bttn = 0;
    buffer.motor1_rpm_pcm = 0; //10;
    buffer.motor2_rpm_pcm = 0;
    buffer.motor3_rpm_pcm = 0;
    buffer.motor4_rpm_pcm = 0;
}
int convert_axis_to_pwm(int axis_value) {
    // Convert the joystick axis value to a PWM value
    // Assuming axis_value is in the range of 0-4095 for a 12-bit ADC
    // and we want to map it to a PWM range of 0-255
    return (axis_value * 255) / 4095;
}
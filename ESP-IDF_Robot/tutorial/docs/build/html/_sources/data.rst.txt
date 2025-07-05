DATA STRUCT
===========

.. code-block:: c

    typedef struct {
        int         x_axis;             // Joystick x-position
        int         y_axis;             // Joystick y-position
        bool        nav_bttn;           // Joystick push button
        bool        led;                // LED ON/OFF state
        uint8_t     motor1_rpm_pwm;     // PWMs for 4 DC motors
        uint8_t     motor2_rpm_pwm;
        uint8_t     motor3_rpm_pwm;
        uint8_t     motor4_rpm_pwm;
    } __attribute__((packed)) sensors_data_t;
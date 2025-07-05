DATA STRUCT
===========

The struct serves as the data payload for sending control signals from transmitting device to the receiver using ESP-NOW.
In addition, it may contain additional data such as telemetry, battery status, etc.

The *sensors_data_t* struct is designed as a data payload that encapsulates all control commands and sensor states relevant to the vehicle's operation.
It's intended to be sent from a transmitting device (like a remote control or master controller) to a receiver (such as a microcontroller onboard the vehicle).

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

When used with communication protocols like ESP-NOW, this struct is **encoded** into a byte stream, then
**transmitted** at regular intervals or in response to user input, and finally
**decoded** on the receiving end to control hardware.

Struct Walkthrough
^^^^^^^^^^^^^^^^^^

*x_axis* and *y_axis* fields capture analog input from a joystick, determining direction and speed.
*nav_bttn* represents a joystick push-button.

*led* allows the transmitter to toggle an onboard LED and is used for status indication (e.g. pairing, battery warning, etc).

*motor1_rpm_pwm* to *motor4_rpm_pwm* provide individual PWM signals to four DC motors.
This enables fine-grained speed control, supports differential drive configurations, and even allows for maneuvering in multi-directional platforms like omni-wheel robots.

Why Use __attribute((packed))?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The *packed* attribute tells the compiler not to add any padding between fields in memory. This makes the struct:

   - Compact
   - Predictable for serialization over protocols like UART or ESP-NOW
   - Ideal for low-latency transmission in embedded systems

This ensures the receiver interprets the exact byte layout you expect, minimizing bandwidth and maximizing compatibility across platforms.
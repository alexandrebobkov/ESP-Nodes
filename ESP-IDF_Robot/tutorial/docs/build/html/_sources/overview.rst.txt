HOW DOES IT WORK?
=================


The BitByteRider RC car is powered by ESP32-C3 Breadboard & Power adapter developmemt board.

Reserved Pins & GPIOs
---------------------

The following table summarizes GPIOs and pins reserved for operations purposes.

The GPIO numbers correspond to those on the ESP32-C3 WROOM microcontroller. The Pin number corresponds to the pin on the Breadboard and Power adapter development board.

x- and y- axis
~~~~~~~~~~~~~~

The **GPIO0** and **GPIO1** assigned to measuring the voltage of x- and y- axis of the Joystick. Lastly, there is a group of GPIO pairs responsible for PWM for DC motors.

Direction and Speed
~~~~~~~~~~~~~~~~~~~

The two DC motors on the left side are wired to the dedicated PWM channels in pairs. This means that PWM channels can control rotation speed and direction of DC motors in pairs (i.e. left and right side).
Consequently, only four PWM channels are required for controlling the direction of the RC car. 
Based on this constraint, the RC car can only move front, back, and rotate left and right. Any other movements are not possible (i.e. diagonal).

A pair of PWM channels are required for defining rotation speed and direction of the DC motors on each side.
In particular, **GPIO6** and **GPIO5** provide PWM to the left- and right- side DC motors to rotate in a **clockwise** direction.
Similarly, **GPIO4** and **GPIO7** provide PWM to the left- and right- side DC motors to rotate in a **counter-clockwise** direction.
Changing PWM on each channel determines the speed and direction of the RC car.

.. raw:: html

   <br/><br/><br/><br/>

+------+-----+---------------------------------------------------------+----------------+
| GPIO | Pin | Function                                                | Notes          |
+======+=====+=========================================================+================+
| 0    | 16  | Joystick x-axis                                         | ADC1_CH0       |
+------+-----+---------------------------------------------------------+----------------+
| 1    | 15  | Joystick y-axis                                         | ADC1_CH1       |
+------+-----+---------------------------------------------------------+----------------+
| 8    | 5   | Joystick push button                                    |                |
+------+-----+---------------------------------------------------------+----------------+
| 6    | 4   | PWM for clockwise rotation of left-side motors          | LEDC_CHANNEL_1 |
+------+-----+---------------------------------------------------------+----------------+
| 5    | 3   | PWM for clockwise rotation of right-side motors         | LEDC_CHANNEL_0 |
+------+-----+---------------------------------------------------------+----------------+
| 4    | 2   | PWM for counter-clockwise rotation of right-side motors | LEDC_CHANNEL_2 |
+------+-----+---------------------------------------------------------+----------------+
| 7    | 6   | PWM for counter-clockwise rotation of left-side motors  | LEDC_CHANNEL_3 |
+------+-----+---------------------------------------------------------+----------------+

Fusion of Software & Hardware
-----------------------------

The *struct* for storing motors PWM values.

.. code-block:: c

    struct motors_rpm {
        int motor1_rpm_pwm;
        int motor2_rpm_pwm;
        int motor3_rpm_pwm;
        int motor4_rpm_pwm;
    };

The function for updating motors' PWM values.

.. code-block:: c

    // Function to send data to the receiver
    void sendData (void) {
        sensors_data_t buffer;              // Declare data struct

        buffer.crc = 0;
        buffer.x_axis = 0;
        buffer.y_axis = 0;
        buffer.nav_bttn = 0;
        buffer.motor1_rpm_pwm = 0;
        buffer.motor2_rpm_pwm = 0;
        buffer.motor3_rpm_pwm = 0;
        buffer.motor4_rpm_pwm = 0;

        // Display brief summary of data being sent.
        ESP_LOGI(TAG, "Joystick (x,y) position ( 0x%04X, 0x%04X )", (uint8_t)buffer.x_axis, (uint8_t)buffer.y_axis);  
        ESP_LOGI(TAG, "pwm 1, pwm 2 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.pwm, (uint8_t)buffer.pwm);
        ESP_LOGI(TAG, "pwm 3, pwm 4 [ 0x%04X, 0x%04X ]", (uint8_t)buffer.pwm, (uint8_t)buffer.pwm);

        // Call ESP-NOW function to send data (MAC address of receiver, pointer to the memory holding data & data length)
        uint8_t result = esp_now_send(receiver_mac, &buffer, sizeof(buffer));

        // If status is NOT OK, display error message and error code (in hexadecimal).
        if (result != 0) {
            ESP_LOGE("ESP-NOW", "Error sending data! Error code: 0x%04X", result);
            deletePeer();
        }
        else
            ESP_LOGW("ESP-NOW", "Data was sent.");
    }

Schematic
---------

.. image:: _static/ESP-IDF_Robot_schematic.png
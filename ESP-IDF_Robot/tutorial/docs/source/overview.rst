HOW DOES IT WORK?
=================


The BitByteRider RC car is powered by ESP32-C3 Breadboard & Power adapter developmemt board.

Reserver Pins & GPIOs
---------------------

The table below summarizes GPIOs and pins reserved for operations purposes.

GPIO number corresponds to that on the ESP32-C3 WROOM microcontroller. The Pin number corresponds to the pin on the Breadboard and Power adapter development board.
The GPIO0 and GPIO1 assigned to measuring the voltage of x- and y- axis of the Joystick. Lastly, there is a group of GPIO pairs responsible for PWM for DC motors.
The two DC motors on the left side are wired to the same PWM channel, as well as the two DC motors on the right side. Consequently, only four PWM channels are required for controlling the direction of the RC car.
A pair of PWM channels are required for defining rotation speed and direction of the DC motors on each side.

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

Schematic
---------

.. image:: _static/ESP-IDF_Robot_schematic.png
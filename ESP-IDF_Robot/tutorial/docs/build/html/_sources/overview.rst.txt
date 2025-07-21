HOW DOES IT WORK?
=================

The bitByteRider RC car is powered by ESP32-C3 bitBoard. The Schematic and KiCAd PCB board files are available 
on GitHub_: https://github.com/alexandrebobkov/ESP32-C3_Breadboard-Adapter

The bitByteRider RC car operates using two main units: the *transmitter*, which reads and sends the joystick's X and Y values, and 
the *receiver*, which interprets these values and converts them into PWM signals to control the DC motors. Both units communicate 
via **ESP-NOW**, a low-latency, connectionless wireless protocol that requires no Wi-Fi network or pairing.

In addition to enabling real-time control, using ESP-NOW introduces key networking concepts such as **data encapsulation** and 
structured communication. By using data structures to group control variables, you gain hands-on experience with how information 
is packaged and transmitted, laying the groundwork for understanding the fundamentals of network communication in embedded systems.

The joystick used in the bitByteRider RC car remote unit outputs analog voltages ranging from 0V to 3.3V on both the x- and y-axes, 
depending on the position of the joystick. These voltage levels are read by the ESP32-C3's ADC (Analog-to-Digital Converter) inputs.

When the joystick is in its neutral (centred) position, the ADC inputs on the ESP32-C3 receive approximately 1.65V on both axes. 
This midpoint voltage is interpreted and interpolated into a PWM (Pulse Width Modulation) value of 0, indicating no movement or 
motor activity.

As the joystick is pushed to its maximum positions along the x- and y-axis, the voltage increases up to 3.3V. This maximum voltage 
is interpolated to a PWM value of 1024, which corresponds to a 100% duty cycle on the receiver side, resulting in full-speed 
operation of the DC motors.

To transmit control data, the X and Y axis values are encapsulated in a C struct, along with the receiver's **MAC** address, and sent 
wirelessly using ESP-NOW. This protocol enables low-latency, connectionless communication between the transmitter and receiver 
without requiring a Wi-Fi network or pairing.

Upon reception, the RC car's receiver decapsulates the data, extracts the joystick values, and interpolates them into PWM 
signals. These signals are then used to control the rotation speeds of the DC motors, enabling smooth and responsive remote control. 

This process not only facilitates real-time control but also introduces you to key networking concepts such as data 
encapsulation, data structs, and the fundamentals of wireless data transmission in embedded systems.

.. admonition:: What is encapsulation?

    Encapsulation refers to the process of organizing and packaging data into a structured format before it is transmitted between 
    devices. This is a fundamental concept in networking and communication protocols, including those used in IoT systems.

.. _GitHub: https://github.com/alexandrebobkov/ESP32-C3_Breadboard-Adapter

Reserved Pins & GPIOs
---------------------

The following table summarizes GPIOs and pins reserved for operations purposes.

The GPIO numbers correspond to those on the ESP32-C3 WROOM microcontroller. The Pin number corresponds to the pin on the Breadboard and Power adapter development board.

Reading the Joystick x- and y- axis
~~~~~~~~~~~~~~~~~~~~~~

To determine the position of the Joystick, the BitRider RC car uses ADC to measure voltage on two GPIOs connected to the joystick 
x- and y- axis potentionometers (**GPIO0** and **GPIO1**).

Controlling the Direction and Speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To set any desired speed of BiteRider RC car, the *ESP32-C3 Breadboard Adapter DevBoard* uses PWM to control the rotation speed
of DR motors. Similarly, to set the direction of the RC car, the rotation speed of corresponding DC motors is changed as required.

Due to the design and limited number of available GPIOs, the *ESP32-C3 Breadboard DevBoard* can control rotation speed and direction 
of DC motors in pairs only (i.e. left and right side). Consequently, this means that the four PWM channels used for controlling the 
direction of the RC car.

Based on this constraint, the RC car can only move front, back, and turn/rotate left and right. Any other movements are not 
possible (i.e. diagonal or sideways).

+--------------------------+-----------+
| PWM of DC Motors         | Direction |
+--------------------------+-----------+
| PWM(left) = PWM(right)   | Straight  |
+--------------------------+-----------+
| PWM(left) > PWM(right)   | Left      |
+--------------------------+-----------+
| PWM(left) < PWM(right)   | Right     |
+--------------------------+-----------+

.. admonition:: What is PWM?

    **PWM** stands for Pulse Width Modulation. It is a technique used to simulate analog voltage levels using discrete digital signals. It works by 
    rapidly switching a digital GPIO pin between HIGH (on) and LOW (off) states at a fixed frequency (often, at base frequency of 5 kHz). 
    The duty cycle—the percentage of time the signal is HIGH in one cycle determines the effective voltage delivered to a device.
    A higher duty cycle increases the motor speed, and a lower duty cycle decreases the motor speed. This allows for fine-grained speed control 
    without needing analog voltage regulators.

A pair of PWM channels are used per DC motor for defining their rotation speed and direction on each side.
In particular, **GPIO6** and **GPIO5** provide PWM to the left- and right- side DC motors to rotate in a **clockwise** direction.
Similarly, **GPIO4** and **GPIO7** provide PWM to the left- and right- side DC motors to rotate in a **counter-clockwise** direction.
Changing PWM on each channel determines the speed and direction of the RC car.

The table below summarizes the GPIO pins used for PWM to control the direction of the DC motors in the remote-controlled car.

+-----------+-------+---------------------------------------+----------+
| GPIOs     | State | Description                           | Function |          
+===========+=======+=======================================+==========+
| GPIO6,    | PWM   | Left & Right DC Motors spin           | Forward  |
| GPIO4     |       | clockwise                             |          |
+-----------+-------+---------------------------------------+----------+
| GPIO5,    | PWM   | Left & Right DC Motors spin           | Reverse  |
| GPIO7     |       | counterclockwise                      |          |
+-----------+-------+---------------------------------------+----------+
| GPIO6,    | PWM   | Left DC Motors spin clockwise.        | Left     |
| GPIO7     |       | Right DC Motors spin counterclockwise |          |
+-----------+-------+---------------------------------------+----------+
| GPIO4,    | PWM   | Left DC Motors spin counterclockwise. | Right    |
| GPIO5     |       | Right DC Motors spin clockwise        |          |
+-----------+-------+---------------------------------------+----------+

The following images illustrate various PWM duty cycles registered by oscilloscope (duty cycles 0%, 48% and 91%, resp.).

.. figure:: _static/ESP-IDF_Robot_PWM_Duty-0.bmp

    DC Motor PWM duty cycle 0%

.. figure:: _static/ESP-IDF_Robot_PWM_Duty-50.bmp

    DC Motor PWM duty cycle 47.6%

.. figure:: _static/ESP-IDF_Robot_PWM_Duty-95.bmp
    
    DC Motor PWM duty cycle 90.8%

.. raw:: html

   <br/><br/><br/><br/>

Fusion of Software with Hardware
--------------------------------

On one hand, we have the hardware designed so that the joystic x- and y- axis, and DC motors are wired to the proper GPIOs on the
ESP32-C3 WROOM microcontroller. On the other hand, we have the software that reads the joystick x- and y- axis, sends the data 
to the receiver device, and converts that to PWM values on the receiver device.

In essense, the direction and speed of the bitByte Rider car is controlled by the two variables. On the remote controller device, 
the joystic x- and y- axis values are sent to the receiver device in a raw format (i.e. analog voltages, "as-is"). On the receover 
device, these two values are converted to the two PWM values; one for each pair of DC motors on left and right side.

When the joystick is pushed forward, the X-axis voltage remains at 1.65V (neutral), while the Y-axis voltage rises to 3.3V. The 
receiver on the RC car interprets this input and generates 100% PWM duty cycle signals on both sides, driving the car forward at 
full speed.

Similarly, when the joystick is pushed fully to the left or right, the X-axis voltage shifts while the Y-axis remains neutral. For a 
left turn, the receiver translates the signal into 100% PWM on the left-side motors and 0% on the right-side motors, causing the car 
to pivot. The opposite occurs for a right turn, with 100% PWM on the right and 0% on the left, enabling precise directional control.

The table below summarizes the reserved GPIOs. These GPIOs are hard-wired to the corresponding components, and hard-coded in the 
corresponding functions. For example, the GPIOs 0 and 1 are hard-wired to the joystick x- and y- axis, respectively; and, hard-coded
to read analog values and store them in the corresponding x- and y- variables.

+------+-----+---------------------------------------------------------+----------------+
| GPIO | Pin | Function                                                | Notes          |
+======+=====+=========================================================+================+
| 0    | 16  | Joystick x-axis                                         | ADC1_CH0       |
+------+-----+---------------------------------------------------------+----------------+
| 1    | 15  | Joystick y-axis                                         | ADC1_CH1       |
+------+-----+---------------------------------------------------------+----------------+
| 8    | 5   | Joystick push button                                    | NC             |
+------+-----+---------------------------------------------------------+----------------+
| 6    | 4   | PWM for clockwise rotation of left-side motors          | LEDC_CHANNEL_1 |
+------+-----+---------------------------------------------------------+----------------+
| 5    | 3   | PWM for clockwise rotation of right-side motors         | LEDC_CHANNEL_0 |
+------+-----+---------------------------------------------------------+----------------+
| 4    | 2   | PWM for counter-clockwise rotation of right-side motors | LEDC_CHANNEL_2 |
+------+-----+---------------------------------------------------------+----------------+
| 7    | 6   | PWM for counter-clockwise rotation of left-side motors  | LEDC_CHANNEL_3 |
+------+-----+---------------------------------------------------------+----------------+

The struct used to store motor PWM values is shown below. While the bitByteRider RC car can be effectively controlled using 
just two PWM signals—one for each side—the structure is designed to hold four values, allowing room for future enhancements. This 
forward-thinking design supports potential upgrades such as improved maneuverability, individual wheel control, or advanced driving 
modes, making the system more adaptable and scalable for future development.

.. code-block:: c

    struct motors_rpm {
        int motor1_rpm_pwm;
        int motor2_rpm_pwm;
        int motor3_rpm_pwm;
        int motor4_rpm_pwm;
    };

On the transmitter`` device, the PWM values for the DC motors are send to the receover using the following function. The variable
**receiver_mac** stores the MAC address of the receiver device (ESP32-C3 bitBoard on the RC car).

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

This function is invoked by a dedicated FreeRTOS task every 100 milliseconds, ensuring consistent and timely transmission of 
control data to the receiver device. By leveraging FreeRTOS's precise task scheduling, the system maintains low-latency 
communication and predictable behavior—critical for real-time control in embedded applications.

.. code-block:: c

    // Continous, periodic task that sends data.
    static void rc_send_data_task (void *arg) {

        while (true) {
            if (esp_now_is_peer_exist(receiver_mac))
                sendData();
            vTaskDelay (100 / portTICK_PERIOD_MS);
        }
    }

As data is being sent, the function onDataSent() is called to check & display the status of the data transmission.

.. code-block:: c

    // Call-back for the event when data is being sent
    void onDataSent (uint8_t *mac_addr, esp_now_send_status_t status) {
        ESP_LOGW(TAG, "Packet send status: 0x%04X", status);
    }

    ... ... ...
    ... ... ...

On the receiver device, the data is saved in the variables by the call-back function onDataReceived().

.. code-block:: c
    
    // Call-back for the event when data is being received
    void onDataReceived (uint8_t *mac_addr, uint8_t *data, uint8_t data_len) {

        buf = (sensors_data_t*)data;                            // Allocate memory for buffer to store data being received
        ESP_LOGW(TAG, "Data was received");
        ESP_LOGI(TAG, "x-axis: 0x%04x", buf->x_axis);
        ESP_LOGI(TAG, "x-axis: 0x%04x", buf->y_axis);
        ESP_LOGI(TAG, "PWM 1: 0x%04x", buf->motor1_rpm_pwm);
    }

The rc_send_data_task() function runs every 0.1 second to transmit the data to the receiver.


Schematic
---------

.. image:: _static/ESP-IDF_Robot_schematic.png
OVERVIEW
============

At the heart of this project is ESP32-C3 IoT Link, which enables a customizable remote-controlled car to respond to real-time 
control inputs, handle speed adjustments, directional changes, and even extend features like measuring system telemetry values
such as voltage, current, temperature, etc. The foundational setup uses ESP-NOW for transmitter and receiver devices, 
allowing you to control the car's behaviour wirelessly. While the design and physical appearance of the RC car can vary wildly 
depending on your creativity and available hardware, the control system remains the same. To facilitate wireless communication 
between devices, the system employs IoT Link powered by ESP-NOW, which is a lightweight and connection-free protocol ideal for fast, 
low-latency data transmission between ESP32 microcontrollers.

The control values are sent wirelessly to the remotely controlled car using the ESP-NOW protocol. ESP-NOW enables fast and efficient 
communication between ESP32 devices without the need for a Wi-Fi router, network, or pairing. The provided tutorial demonstrates a 
functional setup where a transmitter encapsulates and sends data to a receiver to define the car's speed and direction, forming the 
core communication loop. While the baseline implementation focuses on movement, additional features like lights, sensors, or 
telemetry can easily be integrated by expanding the source code. This modular design gives you the freedom to customize both the 
appearance and behaviour of your RC car, resulting in endless creative possibilities.

ABSTRACT
--------

To enable real-time remote operation of the RC car, the system translates joystick x- and y- axis inputs into PWM (Pulse Width Modulation) signals that control the DC motors. 
These PWM values are stored in a predefined data structure, which is then transmitted wirelessly using ESP-NOW — a low-latency, connectionless 
communication protocol developed by Espressif. Both the transmitter and receiver modules are based on ESP32-C3 microcontrollers.

On the transmitter side, the joystick's X and Y coordinates are continuously monitored and converted into PWM parameters. These values are packed into the 
data structure and sent via ESP-NOW to the receiver.

The receiver module listens for incoming ESP-NOW packets, extracts the PWM control data, and applies it directly to the DC motors. This communication flow 
allows the RC car to respond instantly to user input, managing speed and direction without any physical connection between the devices.
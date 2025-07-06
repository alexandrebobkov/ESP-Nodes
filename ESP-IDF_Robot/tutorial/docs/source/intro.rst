OVERVIEW
============

At the heart of this project is a customizable remote-controlled car that responds to real-time control inputs, capable of handling speed adjustments, 
directional changes, and even extended features like lights or sensors. The foundational setup uses ESP-NOW for transmitter and receiver devices, 
allowing you to wirelessly guide the car's behaviour. While the design and physical appearance of the RC car can vary wildly depending on your 
creativity and available hardware, the control system remains elegantly efficient. To facilitate wireless communication between devices, the system employs 
ESP-NOW is a lightweight and connection-free protocol ideal for fast, low-latency data transmission between ESP32 microcontrollers. Though ESP-NOW is used under 
the hood, the spotlight remains on the RC car itself: how it moves, adapts, and evolves with your ideas.

An ESP-NOW-based remote controller sends control data wirelessly using the ESP-NOW protocol to the remote-controlled car. ESP-NOW enables fast and 
efficient communication between ESP32 devices without the need for a Wi-Fi router, network, or pairing. The provided tutorial demonstrates a functional 
setup where a transmitter sends data to a receiver to define the car's speed and direction, forming the core communication loop. While the baseline 
implementation focuses on movement, additional features like lights, sensors, or telemetry can easily be integrated by expanding the source code. This 
modular design gives users the freedom to customize both the appearance and behaviour of their RC car, resulting in endless creative possibilities.

ABSTRACT
--------

The PWM parameters that control DC motors are stored in data struct, which then in turn is encapsulated for being ransmitted using ESP-NOW.

For transmitting the parameters, the system employs ESP-NOW, a low-latency, connectionless communication protocol developed by Espressif, to facilitate wireless data exchange between 
the transmitter and receiver modules. Both devices are based on ESP32C3 microcontrollers and maintain a synchronized understanding of the data structure 
to ensure seamless communication.

On the transmitter side, joystick input is continuously read and translated into control values. These values are then encapsulated into the 
predefined data structure and transmitted via ESP-NOW to the receiver.

The receiver module listens for incoming ESP-NOW packets, de-encapsulates the joystick data, and converts the received values into PWM signals. 
These signals are then used to control the speed and direction of the DC motors, enabling real-time remote operation of the vehicle.
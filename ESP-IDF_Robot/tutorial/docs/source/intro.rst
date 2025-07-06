INTRODUCTION
============

At the heart of this project is a customizable remote-controlled car that responds to real-time control inputs — capable of handling speed adjustments, 
directional changes, and even extended features like lights or sensors. The foundational setup uses a transmitter and receiver architecture, 
allowing you to wirelessly guide the car's behavior. While the design and physical appearance of the RC car can vary wildly depending on your 
creativity and hardware, the control system remains elegantly efficient. To facilitate wireless communication between devices, the system employs 
ESP-NOW, a lightweight and connection-free protocol ideal for fast, low-latency data transmission between ESP32 boards. Though ESP-NOW is used under 
the hood, the spotlight remains on the RC car itself: how it moves, adapts, and evolves with your ideas.

An ESP-NOW-based remote-controlled car system transmits control data wirelessly using the ESP-NOW protocol, which enables fast and efficient 
communication between ESP32 devices without the need for Wi-Fi pairing. The provided tutorial demonstrates a functional setup where a transmitter 
sends data to a receiver to define the car's speed and direction — forming the core communication loop. While the baseline implementation focuses on 
movement, additional features like lights, sensors, or telemetry can easily be integrated by expanding the code. This modular design gives users the 
freedom to customize both the appearance and behavior of their RC car, resulting in endless creative possibilities.
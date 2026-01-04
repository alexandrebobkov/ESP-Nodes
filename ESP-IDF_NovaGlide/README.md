subsystems/
├── motors/          ✅ LEDC PWM control with joystick mixing
├── adc/             ✅ Joystick ADC reading
├── sensors/         ✅ Temperature, INA219, Ultrasonic
├── connectivity/    ✅ WiFi, ESP-NOW, MQTT
├── controls/        ✅ Joystick mixing algorithm
└── ui/              ✅ LED blinking, button handling

main/
├── ESP-IDF_NovaGlide.c  ✅ Clean main application
├── system_init.c        ✅ NVS initialization
├── scheduler.c          ✅ Unified update loop
└── control_task.c       ✅ Motor control task

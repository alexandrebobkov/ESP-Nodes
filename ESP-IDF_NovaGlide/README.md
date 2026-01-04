# ESP-IDF NovaGlide RC Robot

A modular robot control system for ESP32-C3 with wireless control (ESP-NOW), telemetry (MQTT), and comprehensive sensor integration.

```
main/
├── ESP-IDF_NovaGlide.c  ✅ Clean main application
├── system_init.c        ✅ NVS initialization
├── scheduler.c          ✅ Unified update loop
└── control_task.c       ✅ Motor control task

subsystems/
├── motors/          ✅ LEDC PWM control with joystick mixing
├── adc/             ✅ Joystick ADC reading
├── sensors/         ✅ Temperature, INA219, Ultrasonic
├── connectivity/    ✅ WiFi, ESP-NOW, MQTT
├── controls/        ✅ Joystick mixing algorithm
└── ui/              ✅ LED blinking, button handling
```

## 🏗️ Project Structure

```
ESP-IDF_NovaGlide/
├── main/
│   ├── ESP-IDF_NovaGlide.c    # Main application entry point
│   ├── system_init.c/h        # System initialization (NVS)
│   ├── scheduler.c/h          # Unified subsystem update loop
│   └── control_task.c/h       # Motor control task (joystick → motors)
│
└── subsystems/
    ├── motors/                # Motor PWM control
    │   ├── motors.c/h
    │   └── CMakeLists.txt
    │
    ├── adc/                   # Joystick ADC reading
    │   ├── adc.c/h
    │   └── CMakeLists.txt
    │
    ├── sensors/               # All sensor drivers
    │   ├── temp_sensor.c/h        # ESP32-C3 internal temperature
    │   ├── ina219_sensor.c/h      # Battery voltage/current monitor
    │   ├── ultrasonic_sensor.c/h  # Distance sensor
    │   └── CMakeLists.txt
    │
    ├── connectivity/          # Wireless communication
    │   ├── wifi_sys/              # WiFi initialization
    │   ├── espnow_sys/            # ESP-NOW wireless control
    │   └── mqtt_sys/              # MQTT telemetry
    │
    ├── controls/              # Control algorithms
    │   ├── joystick.c/h           # Joystick mixing algorithm
    │   └── CMakeLists.txt
    │
    └── ui/                    # User interface (LED, buttons)
        ├── ui.c/h
        └── CMakeLists.txt
```

## 🔑 Key Components

### Motors Subsystem
**Location:** `subsystems/motors/`

**Purpose:** Controls 4 DC motors using LEDC PWM (left/right, forward/reverse)

**Key Functions:**
- `motor_system_init()` - Initialize LEDC timers and channels
- `motor_set_pwm(left, right)` - Set signed PWM values (-8191 to +8190)
- `motor_update()` - Apply PWM to hardware (called by scheduler)

**GPIO Pins:**
- Motor 1 (Left Forward): GPIO 6
- Motor 2 (Right Forward): GPIO 5
- Motor 3 (Left Reverse): GPIO 4
- Motor 4 (Right Reverse): GPIO 7

### ESP-NOW Subsystem
**Location:** `subsystems/connectivity/espnow_sys/`

**Purpose:** Receives wireless joystick data from remote control

**Key Functions:**
- `espnow_system_init()` - Initialize ESP-NOW
- `espnow_recv_cb()` - Callback when data received
- Data structure: `sensors_data_t` with x_axis, y_axis values

### MQTT Subsystem
**Location:** `subsystems/connectivity/mqtt_sys/`

**Purpose:** Publishes robot telemetry to MQTT broker

**MQTT Topics:**
- `/bitrider/temp` - Internal temperature (°C)
- `/bitrider/battery_voltage` - Battery voltage (V)
- `/bitrider/sys_current` - System current (mA)
- `/bitrider/sys_power` - System power (mW)
- `/bitrider/pwm_left` - Left motor PWM value
- `/bitrider/pwm_right` - Right motor PWM value

**Configuration:**
- Broker: `mqtt://74.14.210.168`
- WiFi SSID: `IoT_bots`
- Password: `208208208`

### Joystick Control
**Location:** `subsystems/controls/joystick.c`

**Purpose:** Converts joystick X/Y values to differential motor PWM

**Algorithm:**
```c
joystick_mix(x_raw, y_raw, &pwm_left, &pwm_right)
```
- Normalizes joystick input to [-1, +1]
- Applies differential steering (k = 0.4)
- Outputs signed PWM values for each motor side

### Sensors

#### Temperature Sensor
- **Type:** ESP32-C3 internal sensor
- **Update Rate:** Every 5 seconds
- **Range:** 10°C to 50°C

#### INA219 Power Monitor
- **Interface:** I2C (GPIO 3: SDA, GPIO 2: SCL)
- **Measures:** Battery voltage, current, power
- **Update Rate:** Every 2.5 seconds
- **Shunt Resistor:** 100mΩ

#### Ultrasonic Distance Sensor
- **GPIO Pins:** Trigger: GPIO 4, Echo: GPIO 5
- **Update Rate:** Every 1 second
- **Max Range:** 400 cm

### UI Subsystem
**Location:** `subsystems/ui/`

**Features:**
- LED blinking (GPIO 10) - 500ms interval
- Button interrupt handling (GPIO 8)

## 🚀 Building and Flashing

### Prerequisites
- ESP-IDF v5.4.1
- ESP32-C3 development board

### Build
```bash
idf.py set-target esp32c3
idf.py build
```

### Flash and Monitor
```bash
idf.py flash monitor
```

### Exit Monitor
`Ctrl + ]`

## 🔄 System Architecture

### Initialization Flow
1. **System Init** - NVS flash initialization
2. **WiFi Init** - Connect to WiFi network
3. **Subsystem Init** - Initialize all components
4. **Task Creation** - Start control and telemetry tasks
5. **Scheduler Start** - Begin main update loop

### Main Tasks

#### Scheduler Task (50ms / 20Hz)
- Updates all subsystems periodically
- Calls `update()` function for each component
- Priority: 10

#### Control Task (100ms / 10Hz)
- Reads joystick data from ESP-NOW
- Applies mixing algorithm
- Updates motor PWM values
- Priority: 15

#### Telemetry Task (1000ms / 1Hz)
- Collects sensor data
- Publishes to MQTT broker
- Priority: 5

## 📊 Data Flow

```
Remote Control (ESP-NOW)
    ↓
ESP-NOW Subsystem → sensors_data_t (x, y)
    ↓
Control Task → joystick_mix()
    ↓
Motor Subsystem → LEDC PWM
    ↓
Motors (Physical Hardware)

Sensors → Scheduler (periodic updates)
    ↓
Telemetry Task
    ↓
MQTT Broker → Monitoring Dashboard
```

## 🔧 Configuration

### Motor PWM Configuration
- **Frequency:** 7000 Hz
- **Resolution:** 13-bit (0-8191)
- **Range:** -8191 (full reverse) to +8190 (full forward)

### WiFi Configuration
Edit `subsystems/connectivity/wifi_sys/wifi_sys.h`:
```c
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
```

### MQTT Configuration
Edit `subsystems/connectivity/mqtt_sys/mqtt_sys.h`:
```c
#define MQTT_BROKER_URI "mqtt://your_broker_ip"
```

## 🐛 Debugging

### Enable Component Logging
Each subsystem has a TAG for logging:
```c
ESP_LOGI(TAG, "Message");  // Info
ESP_LOGW(TAG, "Warning");  // Warning
ESP_LOGE(TAG, "Error");    // Error
```

### Monitor Motor PWM Values
Motors log PWM values every second:
```
I (1234) MOTORS: PWM L/R: 4000/3500 | M1:4000 M2:3500 M3:0 M4:0
```

### Check Sensor Readings
```
I (5678) TEMP_SENSOR: Temperature: 32.50°C
I (9012) INA219: VBUS: 7.40V, I: 850.00mA, P: 6290.00mW
I (3456) ULTRASONIC: Distance: 45.23 cm
```

## 📝 Adding New Subsystems

1. Create directory: `subsystems/your_subsystem/`
2. Add source files: `your_subsystem.c/h`
3. Create `CMakeLists.txt`:
   ```cmake
   idf_component_register(
       SRCS "your_subsystem.c"
       INCLUDE_DIRS "."
       REQUIRES freertos)
   ```
4. Add to root `CMakeLists.txt`:
   ```cmake
   set(EXTRA_COMPONENT_DIRS 
       ...
       "subsystems/your_subsystem")
   ```
5. Include in `main/scheduler.h` and wire to scheduler

## 📚 Dependencies

### ESP-IDF Components
- freertos
- esp_wifi
- esp_netif
- nvs_flash
- mqtt
- esp_driver_ledc
- esp_driver_gpio
- esp_adc

### External Libraries (Managed Components)
- `esp-idf-lib/i2cdev` - I2C device library
- `esp-idf-lib/ina219` - INA219 power monitor
- `esp-idf-lib/ultrasonic` - HC-SR04 ultrasonic sensor

## 👥 Authors

Alexander Bobkov

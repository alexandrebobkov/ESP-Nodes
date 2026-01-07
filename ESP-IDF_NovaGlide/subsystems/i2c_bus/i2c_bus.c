/**
 * @file i2c_bus.c
 * @brief Centralized I2C Bus Manager for ESP32-C3
 *
 * This module provides a thread-safe, multi-device I2C bus manager that:
 * - Manages a single shared I2C bus accessed by multiple devices
 * - Provides mutex-protected access to prevent bus contention
 * - Tracks registered devices for debugging and management
 * - Offers convenience wrappers for common I2C operations
 *
 * Architecture:
 *
 *     ┌─────────────────────────────────────────┐
 *     │        I2C Bus Manager (This Module)    │
 *     │  ┌────────────────────────────────────┐ │
 *     │  │ Mutex-Protected Shared Bus         │ │
 *     │  └────────────────────────────────────┘ │
 *     │         ▲        ▲        ▲             │
 *     │         │        │        │             │
 *     └─────────┼────────┼────────┼─────────────┘
 *               │        │        │
 *         ┌─────┴──┐ ┌───┴───┐ ┌─┴────────┐
 *         │ INA219 │ │Sensor │ │Ultrasonic│
 *         │ 0x40   │ │ 0x57  │ │  0x??    │
 *         └────────┘ └───────┘ └──────────┘
 *
 * Why a Centralized Manager?
 *
 * Problem: Multiple subsystems (INA219, ultrasonic, future sensors) share
 * one I2C bus. Without coordination:
 * - Bus collisions (two devices transmitting simultaneously)
 * - Data corruption (one device interrupts another's transaction)
 * - Deadlocks (circular wait conditions)
 * - Bus lockup (device holds SDA low indefinitely)
 *
 * Solution: Single manager with mutex protection ensures:
 * - One transaction at a time (serialized access)
 * - Clean transaction boundaries (complete before next starts)
 * - Timeout handling (prevents infinite waits)
 * - Centralized error handling and logging
 *
 * I2C Protocol Primer:
 *
 * I2C uses two wires: SDA (data) and SCL (clock)
 *
 * Write Transaction:
 *   START → [ADDR+W] → ACK → [REG] → ACK → [DATA] → ACK → STOP
 *
 * Read Transaction:
 *   START → [ADDR+W] → ACK → [REG] → ACK →
 *   RESTART → [ADDR+R] → ACK → [DATA] ← ACK → STOP
 *
 * Key Concepts:
 * - Master (ESP32) controls clock and initiates transactions
 * - Slaves (sensors) respond to their addresses
 * - 7-bit addressing: 0x08-0x77 (0x00-0x07 and 0x78-0x7F reserved)
 * - Open-drain bus requires pull-up resistors (4.7kΩ typical)
 * - Bus speed: 100 kHz (Standard) or 400 kHz (Fast Mode)
 *
 * Thread Safety:
 * FreeRTOS mutex ensures atomic transactions. Without it:
 *
 *   Thread A: START → [ADDR]
 *   Thread B: START → [ADDR]  ← BUS COLLISION!
 *
 * With mutex:
 *   Thread A: lock → START → [transaction] → STOP → unlock
 *   Thread B: wait for lock → START → [transaction] → STOP → unlock
 *
 * @author Alexander Bobkov
 * @date January 2026
 */

#include "i2c_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "I2C_BUS";

// ═══════════════════════════════════════════════════════════════
// MODULE-LEVEL STATE (Private, file scope)
// ═══════════════════════════════════════════════════════════════

/**
 * @brief Handle to ESP32 I2C master bus hardware
 *
 * NULL when uninitialized. Once created, this handle represents the
 * physical I2C peripheral (I2C0 on ESP32-C3). All device operations
 * route through this handle.
 */
static i2c_master_bus_handle_t bus_handle = NULL;

/**
 * @brief FreeRTOS mutex for thread-safe bus access
 *
 * Critical for preventing bus contention in multi-threaded systems.
 *
 * Mutex Behavior:
 * - Binary semaphore with ownership (mutex)
 * - Recursive: Same task can take multiple times
 * - Priority inheritance: Prevents priority inversion
 *
 * Usage Pattern:
 *   if (xSemaphoreTake(mutex, timeout) == pdTRUE) {
 *       // Critical section - exclusive bus access
 *       i2c_operation();
 *       xSemaphoreGive(mutex);
 *   }
 *
 * Why 100ms timeout?
 * - Typical I2C transaction: 1-10ms
 * - 100ms allows for slow devices or multi-byte transfers
 * - Prevents infinite blocking if a device malfunctions
 */
static SemaphoreHandle_t bus_mutex = NULL;

/**
 * @brief Registry of all devices on the bus
 *
 * Tracks active devices for:
 * - Debugging (what's on the bus?)
 * - Device management (remove by address/handle)
 * - Error tracking (which device failed?)
 *
 * Structure: Fixed-size array (simple, no dynamic allocation)
 * Alternative designs:
 * - Linked list: More flexible, but requires malloc/free
 * - Hash table: Faster lookup, more complex
 * - Fixed array: Simple, predictable, sufficient for typical use
 */
static i2c_device_t device_registry[I2C_MAX_DEVICES];

/**
 * @brief Count of registered devices
 *
 * Used for:
 * - Array bounds checking
 * - Quick "is bus empty?" test
 * - Logging/debugging
 */
static int device_count = 0;

// ═══════════════════════════════════════════════════════════════
// PUBLIC API FUNCTIONS
// ═══════════════════════════════════════════════════════════════

/**
 * @brief Get the I2C bus handle for direct hardware access
 *
 * Use Case: Advanced users who need direct ESP-IDF I2C API access.
 *
 * Warning: Bypasses mutex protection! Caller is responsible for:
 * - Taking the bus mutex manually
 * - Ensuring transaction atomicity
 * - Proper error handling
 *
 * Most users should use the i2c_bus_read/write functions instead.
 *
 * @return Handle to I2C master bus, or NULL if uninitialized
 */
i2c_master_bus_handle_t i2c_bus_get(void) {
    return bus_handle;
}

/**
 * @brief Initialize the I2C bus manager
 *
 * Initialization Sequence:
 *
 * 1. Check if already initialized (idempotent)
 * 2. Create FreeRTOS mutex for thread safety
 * 3. Configure I2C hardware peripheral
 * 4. Clear device registry
 * 5. Scan bus for existing devices (diagnostic)
 *
 * Hardware Configuration:
 * - Port: I2C_NUM_0 (ESP32-C3 has one I2C controller)
 * - SDA/SCL: Defined by I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO
 * - Clock Source: APB clock (default, 80 MHz on ESP32-C3)
 * - Glitch Filter: 7 cycles (filters noise spikes < 87.5ns @ 80MHz)
 * - Internal Pull-ups: Enabled (weak ~45kΩ, external 4.7kΩ recommended)
 *
 * Glitch Filtering:
 * I2C is sensitive to electrical noise. Glitch filter ignores pulses
 * shorter than N clock cycles, preventing false START/STOP conditions.
 * - Too low (< 3): Noise causes spurious transactions
 * - Too high (> 15): May filter legitimate fast edges
 * - 7 cycles: Good balance for most applications
 *
 * Pull-up Resistors:
 * I2C is open-drain, requires pull-ups on SDA and SCL.
 * - Internal pull-ups: Weak (~45kΩ), sufficient for short traces
 * - External pull-ups: Strong (4.7kΩ), required for:
 *   * Long traces (> 10cm)
 *   * Multiple devices
 *   * High capacitance
 *   * Fast mode (400 kHz)
 *
 * @return ESP_OK on success
 * @return ESP_ERR_NO_MEM if mutex creation fails
 * @return ESP-IDF error codes if bus creation fails
 */
esp_err_t i2c_bus_init(void) {
    // Idempotency check: Allow multiple init calls safely
    if (bus_handle != NULL) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    // ───────────────────────────────────────────────────────────
    // STEP 1: Create mutex for thread-safe access
    // ───────────────────────────────────────────────────────────
    bus_mutex = xSemaphoreCreateMutex();
    if (!bus_mutex) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initializing I2C bus...");
    ESP_LOGI(TAG, "  SDA: %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "  SCL: %d", I2C_MASTER_SCL_IO);

    // ───────────────────────────────────────────────────────────
    // STEP 2: Configure I2C hardware
    // ───────────────────────────────────────────────────────────
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,                      // Use I2C controller 0
        .sda_io_num = I2C_MASTER_SDA_IO,            // Data line GPIO
        .scl_io_num = I2C_MASTER_SCL_IO,            // Clock line GPIO
        .clk_source = I2C_CLK_SRC_DEFAULT,          // APB clock (80 MHz)
        .glitch_ignore_cnt = 7,                     // Filter < 87.5ns spikes
        .flags.enable_internal_pullup = true,       // Enable weak pull-ups
    };

    esp_err_t ret = i2c_new_master_bus(&cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C bus: %s", esp_err_to_name(ret));
        // Clean up mutex on failure
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
        return ret;
    }

    // ───────────────────────────────────────────────────────────
    // STEP 3: Initialize device registry
    // ───────────────────────────────────────────────────────────
    memset(device_registry, 0, sizeof(device_registry));
    device_count = 0;

    ESP_LOGI(TAG, "I2C bus initialized");

    // ───────────────────────────────────────────────────────────
    // STEP 4: Scan for existing devices (diagnostic)
    // ───────────────────────────────────────────────────────────
    i2c_bus_scan();

    return ESP_OK;
}

/**
 * @brief Add a device to the I2C bus
 *
 * Registration Process:
 *
 * 1. Validate bus is initialized
 * 2. Check device registry capacity
 * 3. Create ESP-IDF device handle
 * 4. Add to internal registry
 *
 * What is a Device Handle?
 * - Opaque pointer to ESP-IDF's internal device structure
 * - Contains: address, bus reference, timing parameters
 * - Used in all subsequent I2C operations for this device
 *
 * Device Configuration:
 * - Address Length: 7-bit (standard I2C addressing)
 * - Device Address: 0x08-0x77 (0x00-0x07, 0x78-0x7F reserved)
 * - SCL Speed: Defined by I2C_MASTER_FREQ_HZ (typically 100 kHz)
 *
 * Why Register Devices?
 * - Pre-configuration: Set timing parameters once
 * - Error tracking: Know which device caused issues
 * - Resource management: Clean up on deinit
 * - Debugging: List active devices
 *
 * I2C Address Ranges:
 * - 0x00-0x07: Reserved (general call, START byte, etc.)
 * - 0x08-0x77: Valid 7-bit device addresses
 * - 0x78-0x7F: Reserved (10-bit addressing, future use)
 *
 * @param address 7-bit I2C address (0x08-0x77)
 * @param name Human-readable device name (for logging)
 * @param dev_handle Output: Receives created device handle
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if bus not initialized
 * @return ESP_ERR_NO_MEM if device registry is full
 * @return ESP-IDF error codes on device creation failure
 */
esp_err_t i2c_bus_add_device(uint8_t address, const char *name,
                             i2c_master_dev_handle_t *dev_handle) {
    // Validate bus is initialized
    if (!bus_handle) return ESP_ERR_INVALID_STATE;

    // Check registry capacity
    if (device_count >= I2C_MAX_DEVICES) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Adding device '%s' at 0x%02X", name, address);

    // ───────────────────────────────────────────────────────────
    // Configure device parameters
    // ───────────────────────────────────────────────────────────
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,     // Standard 7-bit addressing
        .device_address = address,                  // Device's I2C address
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,        // Bus clock speed
    };

    // Create device handle with ESP-IDF
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        return ret;
    }

    // ───────────────────────────────────────────────────────────
    // Register device in our tracking system
    // ───────────────────────────────────────────────────────────
    device_registry[device_count].address = address;
    device_registry[device_count].dev_handle = *dev_handle;
    device_registry[device_count].is_active = true;
    device_registry[device_count].name = name;
    device_count++;

    return ESP_OK;
}

/**
 * @brief Write data to a device register
 *
 * I2C Register Write Protocol:
 *
 *   START → [ADDR+W] → ACK → [REG] → ACK → [DATA...] → ACK → STOP
 *
 *   Example: Write 0x42 to register 0x10 on device 0x40:
 *     START → 0x80 (0x40<<1 | W) → ACK → 0x10 → ACK → 0x42 → ACK → STOP
 *
 * Operation:
 * 1. Take bus mutex (block if another transaction in progress)
 * 2. Build packet: [reg_addr][data bytes]
 * 3. Transmit to device
 * 4. Release mutex
 *
 * Buffer Size Limit:
 * Uses 128-byte stack buffer for efficiency. For larger transfers,
 * consider dynamic allocation or chunked writes.
 *
 * Mutex Timeout:
 * 100ms timeout prevents infinite blocking if another thread hangs.
 * If timeout occurs, returns ESP_ERR_TIMEOUT.
 *
 * @param dev_handle Device handle from i2c_bus_add_device()
 * @param reg_addr Register address to write to
 * @param data Pointer to data bytes
 * @param len Number of bytes to write
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if dev_handle is NULL
 * @return ESP_ERR_TIMEOUT if mutex not available within 100ms
 * @return ESP_ERR_INVALID_SIZE if len + 1 > 128 bytes
 * @return ESP-IDF I2C error codes on transmission failure
 */
esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    // ───────────────────────────────────────────────────────────
    // Acquire exclusive bus access
    // ───────────────────────────────────────────────────────────
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    // ───────────────────────────────────────────────────────────
    // Build I2C packet: [register][data bytes...]
    // ───────────────────────────────────────────────────────────
    uint8_t buf[128];
    if (len + 1 > sizeof(buf)) {
        xSemaphoreGive(bus_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    buf[0] = reg_addr;              // First byte is register address
    memcpy(&buf[1], data, len);     // Followed by data bytes

    // ───────────────────────────────────────────────────────────
    // Transmit to device
    // ───────────────────────────────────────────────────────────
    esp_err_t ret = i2c_master_transmit(dev_handle, buf, len + 1,
                                        I2C_MASTER_TIMEOUT_MS);

    // ───────────────────────────────────────────────────────────
    // Release bus for other threads
    // ───────────────────────────────────────────────────────────
    xSemaphoreGive(bus_mutex);

    return ret;
}

/**
 * @brief Write a single byte to a device register
 *
 * Convenience wrapper around i2c_bus_write() for single-byte operations.
 *
 * Common use cases:
 * - Setting configuration registers
 * - Triggering measurements
 * - Writing command bytes
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param value Byte value to write
 *
 * @return ESP_OK on success, error codes on failure
 */
esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle,
                             uint8_t reg_addr, uint8_t value) {
    return i2c_bus_write(dev_handle, reg_addr, &value, 1);
}

/**
 * @brief Read data from a device register
 *
 * I2C Register Read Protocol (Combined Transaction):
 *
 *   START → [ADDR+W] → ACK → [REG] → ACK →
 *   RESTART → [ADDR+R] → ACK → [DATA] ← ACK → ... → NACK → STOP
 *
 *   Example: Read 2 bytes from register 0x02 on device 0x40:
 *     START → 0x80 (0x40<<1 | W) → ACK → 0x02 → ACK →
 *     RESTART → 0x81 (0x40<<1 | R) → ACK → [byte1] ← ACK → [byte2] ← NACK → STOP
 *
 * Why RESTART?
 * Combined (write-then-read) transaction prevents other devices from
 * accessing the bus between register address write and data read.
 * This ensures atomicity.
 *
 * Alternative (Separate Transactions):
 *   STOP after register write, START before read
 *   Problem: Another device could access bus in between!
 *
 * Operation:
 * 1. Take bus mutex
 * 2. Transmit register address (write phase)
 * 3. Without releasing bus, receive data (read phase)
 * 4. Release mutex
 *
 * ESP-IDF Implementation:
 * i2c_master_transmit_receive() handles the RESTART automatically.
 * This is cleaner than manually managing two separate transactions.
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address to read from
 * @param data Buffer to receive data
 * @param len Number of bytes to read
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if dev_handle is NULL
 * @return ESP_ERR_TIMEOUT if mutex not available
 * @return ESP-IDF I2C error codes on transaction failure
 */
esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    // ───────────────────────────────────────────────────────────
    // Acquire exclusive bus access
    // ───────────────────────────────────────────────────────────
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    // ───────────────────────────────────────────────────────────
    // Combined transaction: Write register address, then read data
    // ───────────────────────────────────────────────────────────
    // ESP-IDF handles RESTART automatically between write and read phases
    esp_err_t ret = i2c_master_transmit_receive(dev_handle,
                                                &reg_addr, 1,    // Write phase
                                                data, len,       // Read phase
                                                I2C_MASTER_TIMEOUT_MS);

    // ───────────────────────────────────────────────────────────
    // Release bus
    // ───────────────────────────────────────────────────────────
    xSemaphoreGive(bus_mutex);

    return ret;
}

/**
 * @brief Read a single byte from a device register
 *
 * Convenience wrapper around i2c_bus_read() for single-byte reads.
 *
 * Common use cases:
 * - Reading status registers
 * - Reading device ID
 * - Polling for completion flags
 *
 * @param dev_handle Device handle
 * @param reg_addr Register address
 * @param value Output: Receives byte value
 *
 * @return ESP_OK on success, error codes on failure
 */
esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr, uint8_t *value) {
    return i2c_bus_read(dev_handle, reg_addr, value, 1);
}

/**
 * @brief Check if a device is present on the bus
 *
 * Detection Method:
 * Send the device address and check for ACK response.
 *
 *   START → [ADDR+W] → ACK? → STOP
 *
 * - If device responds with ACK: Device is present
 * - If no ACK (NACK or timeout): Device is absent
 *
 * Use Cases:
 * - Bus scanning (find all devices)
 * - Hot-plug detection (device removed/inserted?)
 * - Diagnostic checks (is sensor working?)
 *
 * Limitations:
 * - Some devices don't ACK until properly initialized
 * - Bus faults can cause false negatives
 * - Doesn't guarantee device is functional, just addressable
 *
 * @param address 7-bit I2C address to probe
 *
 * @return true if device ACKs, false otherwise
 */
bool i2c_bus_device_present(uint8_t address) {
    if (!bus_handle) return false;

    // ESP-IDF probe function: sends address, checks for ACK
    return i2c_master_probe(bus_handle, address, I2C_MASTER_TIMEOUT_MS) == ESP_OK;
}

/**
 * @brief Scan I2C bus for all present devices
 *
 * Scans the valid 7-bit address range (0x08-0x77) and logs found devices.
 *
 * Address Range Explanation:
 * - 0x00-0x07: Reserved addresses (general call, START byte, etc.)
 * - 0x08-0x77: Valid device addresses (112 addresses)
 * - 0x78-0x7F: Reserved (10-bit addressing)
 *
 * Use Cases:
 * - Initial setup: What devices are on the bus?
 * - Debugging: Did the sensor get detected?
 * - Diagnostics: Bus health check
 *
 * Performance:
 * Scans 112 addresses at ~1ms per probe = ~112ms total.
 * This is acceptable for one-time initialization but too slow for
 * runtime polling.
 *
 * @return Number of devices found
 */
int i2c_bus_scan(void) {
    if (!bus_handle) return 0;

    ESP_LOGI(TAG, "Scanning I2C bus...");

    int found = 0;

    // Scan valid 7-bit address range
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_bus_device_present(addr)) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }

    if (!found) {
        ESP_LOGW(TAG, "No I2C devices found");
    }

    return found;
}

/**
 * @brief Remove a device from the bus
 *
 * Cleanup Process:
 * 1. Remove device from ESP-IDF's internal structures
 * 2. Free associated resources
 *
 * Note: Does NOT update our device_registry. This is intentional -
 * the registry is for debugging and typically isn't modified at runtime.
 *
 * Use Case:
 * Hot-unplugging devices (rare in embedded systems).
 * Most applications initialize devices once and never remove them.
 *
 * @param dev_handle Device handle to remove
 *
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_ARG if handle is NULL
 * @return ESP-IDF error codes on failure
 */
esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    return i2c_master_bus_rm_device(dev_handle);
}

/**
 * @brief Shut down the I2C bus manager
 *
 * Cleanup Sequence:
 * 1. Delete I2C master bus (frees hardware resources)
 * 2. Delete mutex (frees FreeRTOS object)
 * 3. Clear state variables
 *
 * Important: Caller must ensure no devices are actively using the bus!
 * Removing the bus while transactions are in progress causes undefined
 * behavior (likely crashes or bus lockup).
 *
 * Proper Shutdown Order:
 * 1. Stop all tasks using I2C
 * 2. Call i2c_bus_remove_device() for all devices
 * 3. Call i2c_bus_deinit()
 *
 * @return ESP_OK on success
 * @return ESP-IDF error codes on bus deletion failure
 */
esp_err_t i2c_bus_deinit(void) {
    if (!bus_handle) return ESP_OK;  // Already deinitialized

    // ───────────────────────────────────────────────────────────
    // Delete I2C master bus (releases hardware)
    // ───────────────────────────────────────────────────────────
    esp_err_t ret = i2c_del_master_bus(bus_handle);
    if (ret != ESP_OK) return ret;

    // ───────────────────────────────────────────────────────────
    // Delete mutex (releases FreeRTOS object)
    // ───────────────────────────────────────────────────────────
    if (bus_mutex) {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
    }

    // ───────────────────────────────────────────────────────────
    // Clear state
    // ───────────────────────────────────────────────────────────
    bus_handle = NULL;
    device_count = 0;

    return ESP_OK;
}

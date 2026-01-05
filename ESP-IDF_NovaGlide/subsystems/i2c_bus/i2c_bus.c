#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "I2C_BUS";

// Global I2C bus handle
static i2c_master_bus_handle_t bus_handle = NULL;
static SemaphoreHandle_t bus_mutex = NULL;

// Device registry
static i2c_device_t device_registry[I2C_MAX_DEVICES];
static int device_count = 0;

// Initialize the I2C bus
esp_err_t i2c_bus_init(void) {
    if (bus_handle != NULL) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    // Create mutex
    bus_mutex = xSemaphoreCreateMutex();
    if (bus_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initializing I2C bus...");
    ESP_LOGI(TAG, "  SCL: GPIO %d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "  SDA: GPIO %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "  Frequency: %d Hz", I2C_MASTER_FREQ_HZ);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
        return ret;
    }

    // Initialize device registry
    memset(device_registry, 0, sizeof(device_registry));
    device_count = 0;

    ESP_LOGI(TAG, "I2C bus initialized successfully");

    // Scan for devices
    int found = i2c_bus_scan();
    ESP_LOGI(TAG, "Found %d device(s) on the bus", found);

    return ESP_OK;
}

// Add a device to the bus
esp_err_t i2c_bus_add_device(uint8_t address, const char *name, i2c_master_dev_handle_t *dev_handle) {
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized. Call i2c_bus_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (device_count >= I2C_MAX_DEVICES) {
        ESP_LOGE(TAG, "Maximum number of devices (%d) reached", I2C_MAX_DEVICES);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Adding device '%s' at address 0x%02X", name ? name : "Unknown", address);

    // Check if device is present
    if (!i2c_bus_device_present(address)) {
        ESP_LOGW(TAG, "Device at 0x%02X not responding", address);
        // Continue anyway - device might be in sleep mode
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register device
    device_registry[device_count].address = address;
    device_registry[device_count].dev_handle = *dev_handle;
    device_registry[device_count].is_active = true;
    device_registry[device_count].name = name;
    device_count++;

    ESP_LOGI(TAG, "Device '%s' added successfully (%d/%d)",
             name ? name : "Unknown", device_count, I2C_MAX_DEVICES);

    return ESP_OK;
}

// Write data to device (with mutex)
esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len) {
    if (dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bus_mutex == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire I2C mutex for write");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t write_buf[len + 1];
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);

    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, len + 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write failed: %s", esp_err_to_name(ret));
    }

    // Release mutex
    xSemaphoreGive(bus_mutex);

    return ret;
}

// Write single byte
esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t value) {
    return i2c_bus_write(dev_handle, reg_addr, &value, 1);
}

// Read data from device (with mutex)
esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len) {
    if (dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bus_mutex == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex
    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire I2C mutex for read");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1,
                                                 data, len, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
    }

    // Release mutex
    xSemaphoreGive(bus_mutex);

    return ret;
}

// Read single byte
esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *value) {
    return i2c_bus_read(dev_handle, reg_addr, value, 1);
}

// Check if device is present
bool i2c_bus_device_present(uint8_t address) {
    if (bus_handle == NULL) {
        return false;
    }

    esp_err_t ret = i2c_master_probe(bus_handle, address, I2C_MASTER_TIMEOUT_MS);
    return (ret == ESP_OK);
}

// Scan I2C bus
int i2c_bus_scan(void) {
    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return 0;
    }

    ESP_LOGI(TAG, "Scanning I2C bus...");
    int found = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_bus_device_present(addr)) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found");
    }

    return found;
}

// Remove device
esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle) {
    if (dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_rm_device(dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Update registry
    for (int i = 0; i < device_count; i++) {
        if (device_registry[i].dev_handle == dev_handle) {
            device_registry[i].is_active = false;
            ESP_LOGI(TAG, "Device '%s' removed", device_registry[i].name);
            break;
        }
    }

    return ESP_OK;
}

// Deinitialize I2C bus
esp_err_t i2c_bus_deinit(void) {
    if (bus_handle == NULL) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing I2C bus...");

    esp_err_t ret = i2c_del_master_bus(bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize: %s", esp_err_to_name(ret));
        return ret;
    }

    if (bus_mutex != NULL) {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
    }

    bus_handle = NULL;
    device_count = 0;

    ESP_LOGI(TAG, "I2C bus deinitialized");
    return ESP_OK;
}

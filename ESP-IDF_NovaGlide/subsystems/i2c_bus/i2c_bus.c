#include "i2c_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "I2C_BUS";

static i2c_master_bus_handle_t bus_handle = NULL;
static SemaphoreHandle_t bus_mutex = NULL;

static i2c_device_t device_registry[I2C_MAX_DEVICES];
static int device_count = 0;

i2c_master_bus_handle_t i2c_bus_get(void) {
    return bus_handle;
}

esp_err_t i2c_bus_init(void) {
    if (bus_handle != NULL) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    bus_mutex = xSemaphoreCreateMutex();
    if (!bus_mutex) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Initializing I2C bus...");
    ESP_LOGI(TAG, "  SDA: %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "  SCL: %d", I2C_MASTER_SCL_IO);

    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&cfg, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2C bus: %s", esp_err_to_name(ret));
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
        return ret;
    }

    memset(device_registry, 0, sizeof(device_registry));
    device_count = 0;

    ESP_LOGI(TAG, "I2C bus initialized");
    i2c_bus_scan();

    return ESP_OK;
}

esp_err_t i2c_bus_add_device(uint8_t address, const char *name,
                             i2c_master_dev_handle_t *dev_handle) {
    if (!bus_handle) return ESP_ERR_INVALID_STATE;
    if (device_count >= I2C_MAX_DEVICES) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "Adding device '%s' at 0x%02X", name, address);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device: %s", esp_err_to_name(ret));
        return ret;
    }

    device_registry[device_count].address = address;
    device_registry[device_count].dev_handle = *dev_handle;
    device_registry[device_count].is_active = true;
    device_registry[device_count].name = name;
    device_count++;

    return ESP_OK;
}

esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                        const uint8_t *data, size_t len) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    uint8_t buf[128];
    if (len + 1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;

    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);

    esp_err_t ret = i2c_master_transmit(dev_handle, buf, len + 1,
                                        I2C_MASTER_TIMEOUT_MS);

    xSemaphoreGive(bus_mutex);
    return ret;
}

esp_err_t i2c_bus_write_byte(i2c_master_dev_handle_t dev_handle,
                             uint8_t reg_addr, uint8_t value) {
    return i2c_bus_write(dev_handle, reg_addr, &value, 1);
}

esp_err_t i2c_bus_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,
                       uint8_t *data, size_t len) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle,
                                                &reg_addr, 1,
                                                data, len,
                                                I2C_MASTER_TIMEOUT_MS);

    xSemaphoreGive(bus_mutex);
    return ret;
}

esp_err_t i2c_bus_read_byte(i2c_master_dev_handle_t dev_handle,
                            uint8_t reg_addr, uint8_t *value) {
    return i2c_bus_read(dev_handle, reg_addr, value, 1);
}

bool i2c_bus_device_present(uint8_t address) {
    if (!bus_handle) return false;
    return i2c_master_probe(bus_handle, address, I2C_MASTER_TIMEOUT_MS) == ESP_OK;
}

int i2c_bus_scan(void) {
    if (!bus_handle) return 0;

    ESP_LOGI(TAG, "Scanning I2C bus...");
    int found = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_bus_device_present(addr)) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
    }

    if (!found) ESP_LOGW(TAG, "No I2C devices found");
    return found;
}

esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle) {
    if (!dev_handle) return ESP_ERR_INVALID_ARG;
    return i2c_master_bus_rm_device(dev_handle);
}

esp_err_t i2c_bus_deinit(void) {
    if (!bus_handle) return ESP_OK;

    esp_err_t ret = i2c_del_master_bus(bus_handle);
    if (ret != ESP_OK) return ret;

    if (bus_mutex) {
        vSemaphoreDelete(bus_mutex);
        bus_mutex = NULL;
    }

    bus_handle = NULL;
    device_count = 0;

    return ESP_OK;
}

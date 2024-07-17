#include "bme280.h"

static esp_err_t bme280_read_raw_temperature(uint8_t device_address, uint8_t *raw_temperature) {}

// REQUESTS DEVICE ID
static esp_err_t bme280_get_id(i2c_cmd_handle_t command, uint8_t device_address) {
    // The sequence of commands corresponds to the BME-280 I2C protocol
    int len = 1; // 1 byte
    uint8_t *data = malloc(len);
    //uint8_t address = 0x76;
    uint8_t register_address = 0xD0;    // Register that stores device ID   
    ESP_ERROR_CHECK(i2c_driver_initialize());
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);  
    command = i2c_cmd_link_create();  
    // Step 1. START
    i2c_master_start(command);
    // Step 2. Send device address, WRITE bit, and wait for acknowledgment
    i2c_master_write_byte(command, device_address << 1 | I2C_MASTER_WRITE, 0x1);
    // Step 3. Specify (write) register address, and wait for acknowledgment
    i2c_master_write_byte(command, register_address, 0x1);
    // Step 4. START
    i2c_master_start(command);         
    // Step 5. Send device address, READ bit, and wait for acknowledgment
    i2c_master_write_byte(command, device_address << 1 | I2C_MASTER_READ, 0x1);
    if (len > 1)
        i2c_master_read(command, data, len-1, 0x0);
    // Step 6. Read one byte of data from register
    i2c_master_read_byte(command, data+len-1, 0x1);
    // Step 8. STOP
    i2c_master_stop(command);
    // Step 7. Read one byte of data from register
    esp_err_t cmd_ret = i2c_master_cmd_begin(I2C_NUM_0, command, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(command);
    return cmd_ret;

    /*if (cmd_ret == ESP_OK) {
        ESP_LOGI(TAG, "Register read success");
        for (int i = 0; i < len; i++) {            
            //printf("0x%02x ", data[i]);
            if (data[i] == 0x60)
                ESP_LOGI(TAG, "Device ID is 0x%X (BME-280)", data[i]);
            else if (data[i] == 0x58)
                ESP_LOGI(TAG, "Device ID is 0x%X (BMP-280)", data[i]);
            else 
                ESP_LOGI(TAG, "Device ID is 0x%X", data[i]);
        }        
    } else if (cmd_ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }*/
}

void init_bme280() {}
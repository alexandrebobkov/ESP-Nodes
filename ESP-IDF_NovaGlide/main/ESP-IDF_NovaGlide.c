void app_main(void)
{
    system_init();

    static ina219_system_t ina;
    static ultrasonic_system_t ultra;

    // WiFi only if you really want it during this test
    wifi_system_init();

    // I2C bus
    ESP_ERROR_CHECK(i2c_bus_init());
    i2c_bus_scan();   // should show 0x40 and 0x57

    // Only init I2C-dependent subsystems you actually use here
    ina219_system_init(&ina);
    ultrasonic_system_init(&ultra);

    // Small power-up settle delay for the ultrasonic module
    vTaskDelay(pdMS_TO_TICKS(200));

    // Minimal raw read loop
    for (;;) {
        uint8_t data[2] = {0};
        esp_err_t err = i2c_master_receive(ultra.dev, data, 2, pdMS_TO_TICKS(200));

        if (err == ESP_OK) {
            uint16_t raw_mm = ((uint16_t)data[0] << 8) | data[1];
            float cm = raw_mm / 10.0f;
            ESP_LOGI("ULTRA_RAW", "raw=%u mm, %.2f cm", raw_mm, cm);
        } else {
            ESP_LOGW("ULTRA_RAW", "raw read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

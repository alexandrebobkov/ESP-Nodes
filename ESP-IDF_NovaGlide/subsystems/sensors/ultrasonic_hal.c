#include "ultrasonic_hal.h"
#include "esp_log.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ULTRA_HAL";

#define RMT_RESOLUTION_HZ 1000000  // 1 MHz → 1 tick = 1 µs
#define MEASUREMENT_TIMEOUT_MS 50  // Maximum time to wait for echo
#define RMT_RX_BUFFER_SIZE 64      // Buffer for received symbols

static bool rmt_rx_callback(rmt_channel_handle_t channel,
                            const rmt_rx_done_event_data_t *edata,
                            void *user_ctx)
{
    ultrasonic_hal_t *self = (ultrasonic_hal_t *)user_ctx;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (edata->num_symbols > 0 && edata->received_symbols != NULL) {
        // Copy the duration immediately while buffer is still valid
        // For HC-SR04, we want the HIGH pulse duration (echo time)
        // This is typically duration0 if the echo pulse is the first level
        uint32_t duration = edata->received_symbols[0].duration0;

        // If your echo is captured across both levels, use:
        // uint32_t duration = edata->received_symbols[0].duration0 +
        //                    edata->received_symbols[0].duration1;

        // Store the result atomically
        self->last_pulse_us = duration;
        self->has_pulse = true;
    }

    return xHigherPriorityTaskWoken == pdTRUE;
}

static void ultrasonic_update_impl(ultrasonic_hal_t *self, TickType_t now)
{
    static TickType_t last = 0;

    // Rate limit to once per 100ms
    if ((now - last) < pdMS_TO_TICKS(100)) {
        return;
    }
    last = now;

    if (self->encoder == NULL) {
        ESP_LOGE(TAG, "ENCODER IS NULL IN UPDATE, SKIPPING");
        return;
    }

    // Reset pulse flag before starting new measurement
    self->has_pulse = false;

    // --- 1. Start RX with our own buffer ---
    static rmt_symbol_word_t rx_buffer[RMT_RX_BUFFER_SIZE];

    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 1000,      // 1 µs minimum
        .signal_range_max_ns = 30000000   // 30 ms maximum
    };

    esp_err_t ret = rmt_receive(self->rmt_rx, rx_buffer, sizeof(rx_buffer), &rx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT receive failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return;
    }

    // Small delay to ensure RX is ready
    vTaskDelay(pdMS_TO_TICKS(1));

    // --- 2. Send TRIG pulse ---
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0
    };

    rmt_symbol_word_t pulse = {
        .level0 = 1,
        .duration0 = 10,   // 10 µs high
        .level1 = 0,
        .duration1 = 10    // 10 µs low (for clean transition)
    };

    ret = rmt_transmit(self->rmt_tx, self->encoder, &pulse, sizeof(pulse), &tx_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT transmit failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = rmt_tx_wait_all_done(self->rmt_tx, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT tx wait failed: %s", esp_err_to_name(ret));
        return;
    }

    // --- 3. Wait for echo response (callback will set has_pulse) ---
    // HC-SR04 max range is ~4m which takes ~23ms round trip
    // Wait up to 50ms for the measurement to complete
    TickType_t start = xTaskGetTickCount();
    while (!self->has_pulse &&
           (xTaskGetTickCount() - start) < pdMS_TO_TICKS(MEASUREMENT_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // --- 4. Convert pulse to distance ---
    if (self->has_pulse) {
        uint32_t us = self->last_pulse_us;

        // HC-SR04 formula: distance (cm) = time (µs) / 58
        // This is derived from: distance = (time * speed_of_sound) / 2
        // where speed_of_sound ≈ 343 m/s = 0.0343 cm/µs
        // So: cm = (µs * 0.0343) / 2 = µs / 58.14
        self->distance_cm = (float)us / 58.0f;

        // Sanity check: HC-SR04 range is 2cm to 400cm
        if (self->distance_cm < 2.0f || self->distance_cm > 400.0f) {
            ESP_LOGW(TAG, "Distance out of range: %.2f cm (pulse: %lu µs)",
                     self->distance_cm, us);
        } else {
            ESP_LOGI(TAG, "Distance: %.2f cm (pulse: %lu µs)",
                     self->distance_cm, us);
        }

        self->has_pulse = false;
    } else {
        ESP_LOGW(TAG, "No echo received (timeout)");
        self->distance_cm = -1.0f;  // Indicate measurement failure
    }

    // --- 5. Re-enable RX channel for next measurement ---
    // After receive completes (success or timeout), channel gets disabled
    // We need to re-enable it for the next cycle
    rmt_disable(self->rmt_rx);
    esp_err_t enable_ret = rmt_enable(self->rmt_rx);
    if (enable_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to re-enable RX: %s", esp_err_to_name(enable_ret));
    }
}

void ultrasonic_hal_init(ultrasonic_hal_t *ultra,
                         gpio_num_t trig_pin,
                         gpio_num_t echo_pin)
{
    ultra->trig_pin = trig_pin;
    ultra->echo_pin = echo_pin;
    ultra->distance_cm = 0.0f;
    ultra->last_pulse_us = 0;
    ultra->has_pulse = false;
    ultra->update = ultrasonic_update_impl;

    // --- TX channel for trigger pulse ---
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = trig_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &ultra->rmt_tx));

    // Use copy encoder (simple and robust)
    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &ultra->encoder));

    ESP_LOGI(TAG, "Created encoder: %p", (void*)ultra->encoder);

    ESP_ERROR_CHECK(rmt_enable(ultra->rmt_tx));

    // --- RX channel for echo pulse ---
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = echo_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .flags.invert_in = false,
        .flags.with_dma = false
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &ultra->rmt_rx));

    // Register RX callback
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_callback
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(ultra->rmt_rx, &cbs, ultra));

    ESP_ERROR_CHECK(rmt_enable(ultra->rmt_rx));

    ESP_LOGI(TAG, "Ultrasonic HAL initialized (TRIG=%d, ECHO=%d)",
             trig_pin, echo_pin);
}

void ultrasonic_hal_deinit(ultrasonic_hal_t *ultra)
{
    if (ultra->rmt_tx) {
        rmt_disable(ultra->rmt_tx);
        rmt_del_channel(ultra->rmt_tx);
        ultra->rmt_tx = NULL;
    }

    if (ultra->rmt_rx) {
        rmt_disable(ultra->rmt_rx);
        rmt_del_channel(ultra->rmt_rx);
        ultra->rmt_rx = NULL;
    }

    if (ultra->encoder) {
        rmt_del_encoder(ultra->encoder);
        ultra->encoder = NULL;
    }

    ESP_LOGI(TAG, "Ultrasonic HAL deinitialized");
}

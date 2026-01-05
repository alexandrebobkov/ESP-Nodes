#include "ultrasonic_system.h"
#include "esp_log.h"

static const char *TAG = "ULTRA_HAL";

// RMT resolution: 1 MHz → 1 tick = 1 µs
#define RMT_RESOLUTION_HZ 1000000

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now)
{
    static TickType_t last = 0;

    // Update at 10 Hz
    if ((now - last) < pdMS_TO_TICKS(100)) {
        return;
    }
    last = now;

    // --- 1. Send 10 µs TRIG pulse ---
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0
    };

    rmt_symbol_word_t pulse = {
        .level0 = 1,
        .duration0 = 10,   // 10 µs
        .level1 = 0,
        .duration1 = 10
    };

    ESP_ERROR_CHECK(rmt_transmit(self->rmt_tx, &pulse, sizeof(pulse), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(self->rmt_tx, portMAX_DELAY));

    // --- 2. Receive echo pulse ---
    rmt_rx_done_event_data_t rx_data;
    size_t received = 0;

    esp_err_t err = rmt_receive(self->rmt_rx, &rx_data, sizeof(rx_data), &received, 50 / portTICK_PERIOD_MS);

    if (err != ESP_OK || received == 0) {
        ESP_LOGW(TAG, "No echo received");
        return;
    }

    // Echo high duration in microseconds
    uint32_t us = rx_data.symbols[0].duration0;

    // Convert to cm (speed of sound: 343 m/s → 29.1 µs per cm round trip)
    self->distance_cm = us / 58.0f;

    ESP_LOGI(TAG, "Distance: %.2f cm", self->distance_cm);
}

void ultrasonic_system_init(ultrasonic_system_t *ultra,
                            gpio_num_t trig_pin,
                            gpio_num_t echo_pin)
{
    ultra->trig_pin = trig_pin;
    ultra->echo_pin = echo_pin;
    ultra->distance_cm = 0.0f;
    ultra->update = ultrasonic_update_impl;

    // --- RMT TX (TRIG) ---
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = trig_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &ultra->rmt_tx));
    ESP_ERROR_CHECK(rmt_enable(ultra->rmt_tx));

    // --- RMT RX (ECHO) ---
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = echo_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 30000000
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &ultra->rmt_rx));
    ESP_ERROR_CHECK(rmt_enable(ultra->rmt_rx));

    ESP_LOGI(TAG, "Ultrasonic HAL initialized (TRIG=%d, ECHO=%d)",
             trig_pin, echo_pin);
}

#include "ultrasonic_hal.h"
#include "esp_log.h"

static const char *TAG = "ULTRA_HAL";

#define RMT_RESOLUTION_HZ 1000000  // 1 MHz → 1 tick = 1 µs

static void ultrasonic_update_impl(ultrasonic_system_t *self, TickType_t now)
{
    static TickType_t last = 0;

    if ((now - last) < pdMS_TO_TICKS(100)) {
        return;
    }
    last = now;

    // --- 1. Send TRIG pulse ---
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0
    };

    rmt_symbol_word_t pulse = {
        .level0 = 1,
        .duration0 = 10,
        .level1 = 0,
        .duration1 = 10
    };

    ESP_ERROR_CHECK(rmt_transmit(self->rmt_tx, self->encoder, &pulse, sizeof(pulse), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(self->rmt_tx, portMAX_DELAY));

    // --- 2. Receive echo pulse ---
    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 30000000
    };

    rmt_rx_done_event_data_t rx_data;
    ESP_ERROR_CHECK(rmt_receive(self->rmt_rx, &rx_data, sizeof(rx_data), &rx_cfg));

    // Extract pulse duration
    if (rx_data.num_symbols > 0) {
        uint32_t us = rx_data.symbols[0].duration0;
        self->distance_cm = us / 58.0f;
        ESP_LOGI(TAG, "Distance: %.2f cm", self->distance_cm);
    } else {
        ESP_LOGW(TAG, "No echo received");
    }
}

void ultrasonic_system_init(ultrasonic_system_t *ultra,
                            gpio_num_t trig_pin,
                            gpio_num_t echo_pin)
{
    ultra->trig_pin = trig_pin;
    ultra->echo_pin = echo_pin;
    ultra->distance_cm = 0.0f;
    ultra->update = ultrasonic_update_impl;

    // --- TX channel ---
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = trig_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &ultra->rmt_tx));

    // --- Encoder ---
    rmt_simple_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&enc_cfg, &ultra->encoder));

    ESP_ERROR_CHECK(rmt_enable(ultra->rmt_tx));

    // --- RX channel ---
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = echo_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &ultra->rmt_rx));
    ESP_ERROR_CHECK(rmt_enable(ultra->rmt_rx));

    ESP_LOGI(TAG, "Ultrasonic HAL initialized (TRIG=%d, ECHO=%d)",
             trig_pin, echo_pin);
}

#include "ultrasonic_hal.h"
#include "esp_log.h"

static const char *TAG = "ULTRA_HAL";

#define RMT_RESOLUTION_HZ 1000000  // 1 MHz → 1 tick = 1 µs

static bool rmt_rx_callback(rmt_channel_handle_t channel,
                            const rmt_rx_done_event_data_t *edata,
                            void *user_ctx)
{
    ultrasonic_hal_t *self = (ultrasonic_hal_t *)user_ctx;

    if (edata->num_symbols > 0 && edata->received_symbols != NULL) {
        // Take the first symbol's high duration as echo time
        self->last_pulse_us = edata->received_symbols[0].duration0;
        self->has_pulse = true;
    }

    // No context switch needed
    return false;
}

static void ultrasonic_update_impl(ultrasonic_hal_t *self, TickType_t now)
{
    ESP_LOGI("ULTRA_HAL", "UPDATE: self=%p encoder=%p", (void*)self, (void*)self->encoder);
    if (self->encoder == NULL) {
        ESP_LOGE("ULTRA_HAL", "ENCODER IS NULL IN UPDATE, SKIPPING");
        return;
    }
    ESP_LOGE("ULTRA_HAL", "update() called, now=%lu", (unsigned long) now);

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
        .duration0 = 10,   // 10 µs
        .level1 = 0,
        .duration1 = 10
    };

    ESP_ERROR_CHECK(rmt_transmit(self->rmt_tx, self->encoder,
                                 &pulse, sizeof(pulse), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(self->rmt_tx, portMAX_DELAY));

    // --- 2. Start RX (async, result via callback) ---
    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 30000000
    };

    ESP_ERROR_CHECK(rmt_receive(self->rmt_rx,
                                NULL,   // use driver-managed buffer
                                0,
                                &rx_cfg));

    // --- 3. If we have a pulse from previous cycle, convert to distance ---
    if (self->has_pulse) {
        uint32_t us = self->last_pulse_us;
        self->distance_cm = (float)us / 58.0f;  // HC-SR04 formula
        self->has_pulse = false;

        ESP_LOGI(TAG, "Distance: %.2f cm", self->distance_cm);
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

    // --- TX channel ---
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = trig_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &ultra->rmt_tx));

    rmt_simple_encoder_config_t enc_cfg = {
        .resolution_hz = RMT_RESOLUTION_HZ,   // 1 MHz, same as TX channel
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&enc_cfg, &ultra->encoder));
    ESP_LOGI(TAG, "Encoder handle = %p", (void*) ultra->encoder);

    ESP_ERROR_CHECK(rmt_enable(ultra->rmt_tx));

    // --- RX channel ---
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = echo_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ
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

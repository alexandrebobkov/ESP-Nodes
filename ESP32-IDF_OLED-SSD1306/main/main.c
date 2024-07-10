#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

static const char *TAG = "ESP32-NODE-Display";

#define I2C_MASTER_SCL_IO           (22)      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           (21)      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_ACKS                    0x1
#define I2C_ACKM                    0x0
#define I2C_NOACKM                  0x1                         // I2C NACK value
#define I2C_WRITE_BIT               I2C_MASTER_WRITE
#define I2C_RED_BIT                 I2C_MASTER_READ
#define I2C_ADDRESS                 0x3C

/*void i2c_master_init(SSD1306_t * dev, int16_t sda, int16_t scl, int16_t reset)
{
	ESP_LOGI(TAG, "Legacy i2c driver is used");
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda,
		.scl_io_num = scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
	};
	ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));

	if (reset >= 0) {
		//gpio_pad_select_gpio(reset);
		gpio_reset_pin(reset);
		gpio_set_direction(reset, GPIO_MODE_OUTPUT);
		gpio_set_level(reset, 0);
		vTaskDelay(50 / portTICK_PERIOD_MS);
		gpio_set_level(reset, 1);
	}
	dev->_address = I2C_ADDRESS;
	dev->_flip = false;

}*/
void app_main(void)
{
	SSD1306_t dev;

	i2c_master_init(&dev, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 4);
	ssd1306_init(&dev, 128, 32);
	ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text_x3(&dev, 0, ".", 1, false);
	vTaskDelay(3000 / portTICK_PERIOD_MS);
}

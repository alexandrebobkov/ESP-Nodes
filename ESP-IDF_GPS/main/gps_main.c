/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "nmea_example.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"

static void read_and_parse_nmea();

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    //nmea_example_init_interface();
    read_and_parse_nmea();

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

static void read_and_parse_nmea()
{
    // Sentence string to be parsed
	//char sentence[] = "$GPGLL,4916.45,N,12311.12,W,225444,A,*1D\r\n";
	//char sentence[] = "$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30\r\n";
	//char sentence[] = "$GPTXT,01,03,02,u-blox ag - www.u-blox.com*50\r\n";
	//char sentence[] = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K\r\n";
	char sentence[] = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n";

	printf("Parsing NMEA sentence: %s", sentence);

	// Pointer to struct containing the parsed data. Should be freed manually.
	nmea_s *data;

	// Parse...
	data = nmea_parse(sentence, strlen(sentence), 0);

	if(NULL == data) {
		printf("Failed to parse sentence!\n");
	}

	if (NMEA_GPGLL == data->type) {
		nmea_gpgll_s *gpgll = (nmea_gpgll_s *) data;

		printf("GPGLL Sentence\n");
		printf("Longitude:\n");
		printf("  Degrees: %d\n", gpgll->longitude.degrees);
		printf("  Minutes: %f\n", gpgll->longitude.minutes);
		printf("  Cardinal: %c\n", (char) gpgll->longitude.cardinal);
		printf("Latitude:\n");
		printf("  Degrees: %d\n", gpgll->latitude.degrees);
		printf("  Minutes: %f\n", gpgll->latitude.minutes);
		printf("  Cardinal: %c\n", (char) gpgll->latitude.cardinal);
	}

	if (NMEA_GPGSA == data->type) {
		nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;

		printf("GPGSA Sentence:\n");
		printf("\tMode: %c\n", gpgsa->mode);
		printf("\tFix:  %d\n", gpgsa->fixtype);
		printf("\tPDOP: %.2lf\n", gpgsa->pdop);
		printf("\tHDOP: %.2lf\n", gpgsa->hdop);
		printf("\tVDOP: %.2lf\n", gpgsa->vdop);
	}

	if (NMEA_GPVTG == data->type) {
		nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

		printf("GPVTG Sentence:\n");
		printf("\tTrack [deg]:   %.2lf\n", gpvtg->track_deg);
		printf("\tSpeed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
		printf("\tSpeed [knots]: %.2lf\n", gpvtg->gndspd_knots);
	}

	if (NMEA_GPTXT == data->type) {
		nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

		printf("GPTXT Sentence:\n");
		printf("\tID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
		printf("\t%s\n", gptxt->text);
	}

	if (NMEA_GPGSV == data->type) {
		nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

		printf("GPGSV Sentence:\n");
		printf("\tNum: %d\n", gpgsv->sentences);
		printf("\tID:  %d\n", gpgsv->sentence_number);
		printf("\tSV:  %d\n", gpgsv->satellites);
		printf("\t#1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
		printf("\t#2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
		printf("\t#3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
		printf("\t#4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
	}

	nmea_free(data);
}

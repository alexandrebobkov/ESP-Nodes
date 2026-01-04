#include "scheduler.h"
#include "esp_log.h"

static const char *TAG = "SCHEDULER";

static void scheduler_task(void *arg) {
    scheduler_t *sched = (scheduler_t *)arg;

    ESP_LOGI(TAG, "Scheduler task started");

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Update all subsystems
        if (sched->motors && sched->motors->update) {
            sched->motors->update(sched->motors, now);
        }
        if (sched->adc && sched->adc->update) {
            sched->adc->update(sched->adc, now);
        }
        if (sched->temp && sched->temp->update) {
            sched->temp->update(sched->temp, now);
        }
        if (sched->ina && sched->ina->update) {
            sched->ina->update(sched->ina, now);
        }
        if (sched->ultra && sched->ultra->update) {
            sched->ultra->update(sched->ultra, now);
        }
        if (sched->mqtt && sched->mqtt->update) {
            sched->mqtt->update(sched->mqtt, now);
        }
        if (sched->espnow && sched->espnow->update) {
            sched->espnow->update(sched->espnow, now);
        }
        if (sched->ui && sched->ui->update) {
            sched->ui->update(sched->ui, now);
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz update rate
    }
}

void scheduler_init(scheduler_t *sched) {
    ESP_LOGI(TAG, "Scheduler initialized");
}

void scheduler_start(scheduler_t *sched) {
    xTaskCreate(scheduler_task, "scheduler", 4096, sched, 10, NULL);
    ESP_LOGI(TAG, "Scheduler started");
}

#include "scheduler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void scheduler_task(void *arg)
{
    scheduler_t *sched = (scheduler_t *)arg;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // --- Call each subsystem's update() if it exists ---
        if (sched->adc    && sched->adc->update)    sched->adc->update(sched->adc, now);
        if (sched->motors && sched->motors->update) sched->motors->update(sched->motors, now);
        if (sched->mqtt   && sched->mqtt->update)   sched->mqtt->update(sched->mqtt, now);
        if (sched->temp   && sched->temp->update)   sched->temp->update(sched->temp, now);
        if (sched->ina    && sched->ina->update)    sched->ina->update(sched->ina, now);
        if (sched->ultra  && sched->ultra->update)  sched->ultra->update(sched->ultra, now);

        // --- Loop at 100 Hz ---
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

void scheduler_init(scheduler_t *sched)
{
    // Nothing to do yet — but this hook is useful for future features
    (void)sched;
}

void scheduler_start(scheduler_t *sched)
{
    xTaskCreate(
        scheduler_task,
        "scheduler",
        4096,
        sched,
        5,
        NULL
    );
}

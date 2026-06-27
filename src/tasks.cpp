#include "tasks.h"

#include "imu.h"
#include "config.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#ifdef ESP32

static void imuTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t lastWake = xTaskGetTickCount();
    TickType_t periodTicks =
        pdMS_TO_TICKS(
            (uint32_t)(CONTROL_DT * 1000.0f)
        );

    if(periodTicks < 1)
    {
        periodTicks = 1;
    }

    while(true)
    {
        imuUpdate(CONTROL_DT);

        vTaskDelayUntil(
            &lastWake,
            periodTicks
        );
    }
}

#endif

void startTasks()
{
#ifdef ESP32

    BaseType_t result =
        xTaskCreatePinnedToCore(
            imuTask,
            "IMU",
            4096,
            NULL,
            3,
            NULL,
            0
        );

    if(result != pdPASS)
    {
        Serial.println("Failed to create IMU task");
    }
    else
    {
        Serial.println("IMU task started");
    }

#endif
}

#include "uart_task.h"
#include "filter_task.h"
#include "main.h"
#include <stdio.h>

static TaskHandle_t xTask_UARTHandle = NULL;

extern UART_HandleTypeDef huart1;

static void vTask_UART_BodyFunction(void *pvParameters)
{
    (void)pvParameters;
    char line[128];

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_UART_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        FusionOutput_t out;
        if (xQueuePeek(xQueue_FusionOutput, &out, 0) != pdTRUE) continue;

        int len = snprintf(line, sizeof(line),
                            "t=%lu roll=%.2f pitch=%.2f yaw=%.2f fault=0x%02X\r\n",
                            (unsigned long)out.timestamp_ms,
                            out.roll_deg, out.pitch_deg, out.yaw_deg,
                            out.fault_mask);

        if (len > 0) {
            /* TODO: перейти на HAL_UART_Transmit_DMA, чтобы не блокировать
             * задачу на время передачи ~128 байт при 115200 бод (~11 мс). */
            HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)len, 20);
        }
    }
}

void Task_UART_Start(void)
{
    if (Task_UART_IsRunning())
        return;

    BaseType_t xReturned = xTaskCreate(
        vTask_UART_BodyFunction,
        TASK_UART_NAME,
        TASK_UART_STACK_SIZE,
        NULL,
        TASK_UART_PRIORITY,
        &xTask_UARTHandle
    );

    if (xReturned != pdPASS) {
        xTask_UARTHandle = NULL;
    }
}

void Task_UART_Stop(void)
{
    if (xTask_UARTHandle != NULL) {
        vTaskDelete(xTask_UARTHandle);
        xTask_UARTHandle = NULL;
    }
}

bool Task_UART_IsRunning(void)
{
    if (xTask_UARTHandle == NULL)
        return false;
    return (eTaskGetState(xTask_UARTHandle) != eDeleted);
}

#include "uart_task.h"
#include "filter_task.h"
#include "main.h"

static TaskHandle_t xTask_UARTHandle = NULL;

extern UART_HandleTypeDef huart1;

/* HAL_GetTick() — 32-битный счётчик мс, переполняется через ~49.7 суток.
 * Для метки времени пакета (uint64_t) расширяем его, отслеживая переполнения —
 * работает надёжно, пока функция вызывается чаще периода переполнения
 * (у нас — каждые 2мс, с огромным запасом). */
static uint64_t GetTick64Ms(void)
{
    static uint32_t s_lastTick = 0;
    static uint32_t s_wrapCount = 0;

    uint32_t tick = HAL_GetTick();
    if (tick < s_lastTick) {
        s_wrapCount++;
    }
    s_lastTick = tick;

    return ((uint64_t)s_wrapCount << 32) | (uint64_t)tick;
}

static void vTask_UART_BodyFunction(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_UART_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        FusionOutput_t out;
        if (xQueuePeek(xQueue_FusionOutput, &out, 0) != pdTRUE) continue;

        IMU_Packet_t packet;
        packet.head = IMU_PACKET_HEAD;
        packet.type = IMU_PACKET_TYPE;
        packet.version = IMU_PROTOCOL_VER;
        packet.timestamp_ms = GetTick64Ms();
        packet.gyro_x_dps = out.gyro_dps[0];
        packet.gyro_y_dps = out.gyro_dps[1];
        packet.gyro_z_dps = out.gyro_dps[2];
        packet.accel_x_g = out.accel_g[0];
        packet.accel_y_g = out.accel_g[1];
        packet.accel_z_g = out.accel_g[2];
        packet.roll_deg = out.roll_deg;
        packet.pitch_deg = out.pitch_deg;
        packet.yaw_deg = out.yaw_deg;
        packet.tail = IMU_PACKET_TAIL;

        /* TODO: перейти на HAL_UART_Transmit_DMA — при 500Гц и 48 байт/пакет
         * блокирующая передача занимает заметную долю 2мс-бюджета задачи
         * (см. расчёт: 48*10 бит / бод). Нужно включить DMA TX для USART1
         * в CubeMX (Connectivity -> USART1 -> DMA Settings -> Add -> USART1_TX). */
        HAL_UART_Transmit(&huart1, (uint8_t *)&packet, sizeof(packet), 5);
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

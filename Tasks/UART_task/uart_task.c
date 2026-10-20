#include "uart_task.h"
#include "filter_task.h"
#include "main.h"

static TaskHandle_t xTask_UARTHandle = NULL;
static volatile uint8_t s_mode = TASK_UART_DEFAULT_MODE;

extern UART_HandleTypeDef huart1;

/* HAL_GetTick() — 32-битный счётчик мс, переполняется через ~49.7 суток.
 * Для метки времени пакета (uint64_t) расширяем его, отслеживая переполнения —
 * работает надёжно, пока функция вызывается чаще периода переполнения. */
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

/* ON_REQUEST пока не реализован (нет приёма команд/0x01-запроса) — в этом
 * случае временно используем период 500Гц, чтобы задача не "зависала". */
static uint32_t ModeToPeriodMs(uint8_t mode)
{
    switch (mode) {
        case PROTOCOL_MODE_100HZ:  return 10;
        case PROTOCOL_MODE_250HZ:  return 4;
        case PROTOCOL_MODE_500HZ:  return 2;
        case PROTOCOL_MODE_1000HZ: return 1;
        default:                   return 2;
    }
}

static void vTask_UART_BodyFunction(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        TickType_t xFrequency = pdMS_TO_TICKS(ModeToPeriodMs(s_mode));
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        FusionOutput_t out;
        if (xQueuePeek(xQueue_FusionOutput, &out, 0) != pdTRUE) continue;
        if (!out.valid || (uint32_t)(HAL_GetTick() - out.timestamp_ms) > 10) continue;

        Protocol_TelemetryData_t telemetry;
        telemetry.timestamp_ms = GetTick64Ms();
        telemetry.gyro_x_dps = out.gyro_dps[0];
        telemetry.gyro_y_dps = out.gyro_dps[1];
        telemetry.gyro_z_dps = out.gyro_dps[2];
        telemetry.accel_x_g = out.accel_g[0];
        telemetry.accel_y_g = out.accel_g[1];
        telemetry.accel_z_g = out.accel_g[2];
        telemetry.roll_deg = out.roll_deg;
        telemetry.pitch_deg = out.pitch_deg;
        telemetry.yaw_deg = out.yaw_deg;

        uint8_t frameBuf[PROTOCOL_OVERHEAD_BYTES + sizeof(Protocol_TelemetryData_t)];
        uint16_t frameLen;
        if (!Protocol_BuildFrame(frameBuf, sizeof(frameBuf), PROTOCOL_TYPE_TELEMETRY,
                                  (const uint8_t *)&telemetry, sizeof(telemetry), &frameLen)) {
            continue;
        }

        /* TODO: перейти на HAL_UART_Transmit_DMA — при 500Гц и ~52 байта/пакет
         * блокирующая передача занимает заметную долю 2мс-бюджета задачи.
         * Нужно включить DMA TX для USART1 в CubeMX (Connectivity -> USART1
         * -> DMA Settings -> Add -> USART1_TX).
         * DO2 включён на время самой передачи — видно её реальную длительность. */
        HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);
        HAL_UART_Transmit(&huart1, frameBuf, frameLen, 5);
        HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
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

void Task_UART_SetMode(uint8_t mode)
{
    s_mode = mode;
}

/* Хост-мок железа, нужного вырезанному из uart_task.c коду приёмника:
 * регистры USART1, блокирующая передача, HAL-тик, сброс, очередь FusionOutput.
 * Реальные алгоритмы (ISR, кольцевой буфер, парсер, диспетчер кадров)
 * подключаются без изменений — мок заменяет только периферию и RTOS.
 * Файл самодостаточен: tests/stubs/stm32f4xx_hal.h не используется, чтобы не
 * конфликтовали объявления HAL-функций. */
#ifndef TEST_UART_HOST_MOCKS_H
#define TEST_UART_HOST_MOCKS_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Protocol_IMU.h"

/* ---- Регистры USART1 ---- */
typedef struct {
    uint32_t SR;
    uint32_t DR;
} MockUsart_t;

typedef void *TaskHandle_t;
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) (ms)
static TickType_t xTaskGetTickCount(void);
static void vTaskDelayUntil(TickType_t *last, TickType_t step);

static MockUsart_t g_uartRegs;
#define USART1         (&g_uartRegs)
#define USART_SR_ORE   (1u << 3)
#define USART_SR_RXNE  (1u << 5)

/* ---- HAL ---- */
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;

/* huart1 намеренно не static: вырезанный код содержит своё
 * extern UART_HandleTypeDef huart1;, и объявление должно быть совместимым. */
UART_HandleTypeDef huart1;

static uint32_t g_tickMs;
static uint32_t HAL_GetTick(void) { return g_tickMs; }

static GPIO_TypeDef g_doPort;
#define DO2_GPIO_Port (&g_doPort)
#define DO2_Pin       13u

static void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }

/* Накопленный вывод устройства: тест разбирает его Protocol_ParseFrame. */
#define MOCK_TX_CAP 1024
static uint8_t g_txBuf[MOCK_TX_CAP];
static uint16_t g_txLen;
static unsigned g_txCount;
static uint32_t g_txTimes[128];
static uint8_t g_txTypes[128];

static HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData,
                                           uint16_t Size, uint32_t Timeout)
{
    (void)huart; (void)Timeout;
    if (g_txCount < 128) {
        g_txTimes[g_txCount] = g_tickMs;
        g_txTypes[g_txCount++] = pData[1];
    }
    for (uint16_t i = 0; i < Size && g_txLen < MOCK_TX_CAP; ++i) {
        g_txBuf[g_txLen++] = pData[i];
    }
    return HAL_OK;
}

static int g_resetCalled;
static void HAL_NVIC_SystemReset(void) { g_resetCalled = 1; }

/* ---- Очередь FusionOutput: Task_UART читает её через xQueuePeek ---- */
typedef FusionOutput_t *QueueHandle_t;
#define pdTRUE 1

static FusionOutput_t g_fusion;
static QueueHandle_t xQueue_FusionOutput = NULL;

static int xQueuePeek(QueueHandle_t q, FusionOutput_t *out, int timeout)
{ (void)timeout; *out = *q; return pdTRUE; }

/* ---- Определения из uart_task.c, нужные помощникам ниже ---- */
void Task_UART_RxIsrHandler(void);
static void ProcessRxBytes(void);

/* ---- Помощники теста ---- */
static void Mock_Reset(void)
{
    memset(&g_uartRegs, 0, sizeof(g_uartRegs));
    memset(g_txBuf, 0, sizeof(g_txBuf));
    g_txLen = 0;
    g_txCount = 0;
    g_resetCalled = 0;
    g_tickMs = 12345u;
    memset(&g_fusion, 0, sizeof(g_fusion));
    /* timestamp_ms сравнивается с (uint32_t)GetTick64Ms(), где младшие 32 бита
     * равны HAL_GetTick(); переполнение счётчика тиков на результат не влияет. */
    g_fusion.timestamp_ms = g_tickMs;
    xQueue_FusionOutput = &g_fusion;
}

/* Эмуляция приёма одного байта: ISR читает SR/DR и пишет в кольцевой буфер. */
static void Mock_RxByte(uint8_t byte)
{
    g_uartRegs.DR = byte;
    g_uartRegs.SR = USART_SR_RXNE;
    Task_UART_RxIsrHandler();
}

static void Mock_RxBytes(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; ++i) Mock_RxByte(data[i]);
}

/* Принимает кадр команды и сразу отдаёт его задаче на разбор. */
static void Mock_SendCommand(uint8_t type, const uint8_t *payload, uint16_t payloadLen)
{
    uint8_t frame[PROTOCOL_OVERHEAD_BYTES + PROTOCOL_RX_MAX_PAYLOAD];
    uint16_t frameLen;
    if (!Protocol_BuildFrame(frame, sizeof(frame), type, payload, payloadLen, &frameLen)) {
        abort();
    }
    Mock_RxBytes(frame, frameLen);
    ProcessRxBytes();
}

/* Принимает произвольные байты (шум, обрезанный кадр) и отдаёт задаче. */
static void Mock_SendRaw(const uint8_t *data, uint16_t len)
{
    Mock_RxBytes(data, len);
    ProcessRxBytes();
}

/* Проверяет, что в выводе лежит ровно один кадр нужного типа с указанным
 * payload'ом (payload == NULL — длина проверяется, содержимое нет). */
static int Mock_HasSingleFrame(uint8_t type, const uint8_t *payload, uint16_t payloadLen)
{
    Protocol_Header_t header;
    const uint8_t *got = NULL;
    if (!Protocol_ParseFrame(g_txBuf, g_txLen, &header, &got)) return 0;
    if (header.length != payloadLen) return 0;
    /* Ровно один кадр: следующий байт после него — начало нового или пусто. */
    if (g_txLen != PROTOCOL_OVERHEAD_BYTES + payloadLen) return 0;
    if (header.type != type) return 0;
    if (payloadLen != 0u && payload != NULL && memcmp(got, payload, payloadLen) != 0) return 0;
    return 1;
}

static int Mock_AckIs(uint8_t ackType, uint8_t status)
{
    Protocol_AckNack_t ack;
    ack.ack_type = ackType;
    ack.status = status;
    return Mock_HasSingleFrame(PROTOCOL_TYPE_ACK_NACK, (const uint8_t *)&ack, sizeof(ack));
}

static int Mock_TelemetryIs(uint8_t expectedStatus)
{
    if (!Mock_HasSingleFrame(PROTOCOL_TYPE_TELEMETRY, NULL,
                             (uint16_t)sizeof(Protocol_TelemetryData_t))) {
        return 0;
    }
    return g_txBuf[5u + offsetof(Protocol_TelemetryData_t, status)] == expectedStatus;
}

#endif /* TEST_UART_HOST_MOCKS_H */

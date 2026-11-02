#include "uart_task.h"
#include "filter_task.h"
#include "main.h"
#include <string.h>
#include <math.h>

static TaskHandle_t xTask_UARTHandle = NULL;
static volatile uint8_t s_mode = TASK_UART_DEFAULT_MODE;

extern UART_HandleTypeDef huart1;

/* Приём от хоста: ISR кладёт байты в кольцевой буфер, тело задачи вычитывает
 * их и подаёт в инкрементальный парсер кадров. Буфер степенного размера,
 * head пишет только ISR, tail — только задача, поэтому состояние гонки
 * отсутствует и счётчик отдельно не ведётся. */
#define TASK_UART_RX_BUF_SIZE 128u
#define TASK_UART_RX_BUF_MASK (TASK_UART_RX_BUF_SIZE - 1u)

static volatile uint8_t s_rxBuf[TASK_UART_RX_BUF_SIZE];
static volatile uint32_t s_rxTimestamp[TASK_UART_RX_BUF_SIZE];
static volatile uint16_t s_rxHead; /* пишет ISR */
static volatile uint16_t s_rxTail; /* пишет задача */
static volatile uint32_t s_rxOverflow; /* байты, потерянные при переполнении */
static Protocol_RxParser_t s_rxParser;

void Task_UART_RxIsrHandler(void)
{
    /* Чтение SR затем DR — штатный способ сброса RXNE, а заодно NE/FE/ORE.
     * Ошибки кадра не останавливают приём: битые байты отфильтрует CRC. */
    uint32_t sr = USART1->SR;
    if ((sr & (USART_SR_RXNE | USART_SR_ORE)) != 0) {
        uint8_t byte = (uint8_t)(USART1->DR & 0xFFu);
        uint16_t next = (uint16_t)((s_rxHead + 1u) & TASK_UART_RX_BUF_MASK);
        if (next != s_rxTail) {
            s_rxBuf[s_rxHead] = byte;
            s_rxTimestamp[s_rxHead] = HAL_GetTick();
            s_rxHead = next;
        } else {
            /* Буфер полон — задача не успевает разбирать. Новый байт
             * теряется, а не затирает несчитанный. */
            s_rxOverflow++;
        }
    }
}

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

/* Одна точка выхода в UART: телеметрия, ACK и ответы команд идут из этой
 * задачи, поэтому блокирующая передача не конкурирует ни с кем и мьютекс
 * не нужен. ISR переданных байтов не трогает. */
static void SendFrame(uint8_t type, const uint8_t *payload, uint16_t payloadLen)
{
    /* Хватает на самый большой кадр: ASCII-сообщение (64 байта payload). */
    uint8_t frameBuf[PROTOCOL_OVERHEAD_BYTES + PROTOCOL_ASCII_MAX_LEN];
    uint16_t frameLen;
    if (!Protocol_BuildFrame(frameBuf, sizeof(frameBuf), type, payload,
                             payloadLen, &frameLen)) {
        return;
    }

    /* TODO: перейти на HAL_UART_Transmit_DMA — при 500Гц и 57 байт/пакет
     * блокирующая передача занимает заметную долю 2мс-бюджета задачи.
     * Нужно включить DMA TX для USART1 в CubeMX (Connectivity -> USART1
     * -> DMA Settings -> Add -> USART1_TX).
     * DO2 включён на время самой передачи — видно её реальную длительность. */
    HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, frameBuf, frameLen, 5);
    HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, GPIO_PIN_RESET);
}

static void SendAckNack(uint8_t ackType, uint8_t status)
{
    Protocol_AckNack_t ack;
    ack.ack_type = ackType;
    ack.status = status;
    SendFrame(PROTOCOL_TYPE_ACK_NACK, (const uint8_t *)&ack, sizeof(ack));
}

static bool ReadFusionOutput(FusionOutput_t *out)
{
    return xQueue_FusionOutput != NULL &&
           xQueuePeek(xQueue_FusionOutput, out, 0) == pdTRUE;
}

/* NULL, invalid, stale or non-finite input still produces an error heartbeat. */
static void PrepareTelemetry(const FusionOutput_t *out, uint64_t now,
                             Protocol_TelemetryData_t *telemetry)
{
    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->timestamp_ms = now;
    telemetry->status = PROTOCOL_TELEMETRY_STATUS_ERROR;
    if (out == NULL || !out->valid ||
        (uint32_t)((uint32_t)now - out->timestamp_ms) > 10) return;
    for (int i = 0; i < 3; ++i)
        if (!isfinite(out->gyro_dps[i]) || !isfinite(out->accel_g[i])) return;
    if (!isfinite(out->roll_deg) || !isfinite(out->pitch_deg) ||
        !isfinite(out->yaw_deg) || !isfinite(out->temperature_c)) return;
    telemetry->gyro_x_dps = out->gyro_dps[0];
    telemetry->gyro_y_dps = out->gyro_dps[1];
    telemetry->gyro_z_dps = out->gyro_dps[2];
    telemetry->accel_x_g = out->accel_g[0];
    telemetry->accel_y_g = out->accel_g[1];
    telemetry->accel_z_g = out->accel_g[2];
    telemetry->roll_deg = out->roll_deg;
    telemetry->pitch_deg = out->pitch_deg;
    telemetry->yaw_deg = out->yaw_deg;
    telemetry->temperature_c = out->temperature_c;
    telemetry->status = PROTOCOL_TELEMETRY_STATUS_OK;
}

static void SendTelemetryOnce(void)
{
    FusionOutput_t out;
    bool available = ReadFusionOutput(&out);
    Protocol_TelemetryData_t telemetry;
    PrepareTelemetry(available ? &out : NULL, GetTick64Ms(), &telemetry);
    SendFrame(PROTOCOL_TYPE_TELEMETRY, (const uint8_t *)&telemetry, sizeof(telemetry));
}

/* Applied before the next periodic-send decision; command ACK stays in RAM only. */
void Task_UART_SetMode(uint8_t mode)
{
    if (!Protocol_ModeIsSupported(mode)) return;
    s_mode = mode;
}

uint8_t Task_UART_GetMode(void)
{
    return s_mode;
}

/* Команды, которые прошивка пока не выполняет. Отвечать NACK честнее, чем
 * молчать или выдавать фиктивный OK: хост видит "не поддерживается". */
static void HandleUnimplemented(uint8_t type)
{
    SendAckNack(type, PROTOCOL_STATUS_ERROR);
}

static void HandleModeConfig(const uint8_t *payload, uint16_t payloadLen)
{
    if (payloadLen != sizeof(Protocol_ModeConfig_t)) {
        SendAckNack(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_ERROR);
        return;
    }
    uint8_t requested = ((const Protocol_ModeConfig_t *)payload)->mode;
    if (!Protocol_ModeIsSupported(requested)) {
        SendAckNack(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_ERROR);
        return;
    }
    Task_UART_SetMode(requested);
    SendAckNack(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_OK);
}

static void HandleReset(const uint8_t *payload, uint16_t payloadLen)
{
    /* Спецификация: правильного кадра достаточно, ответа нет — устройство
     * перезагружается. Магическое число в payload защищает от сброса битым
     * или наводнённым пакетом. */
    if (payloadLen != sizeof(Protocol_ResetRequest_t)) {
        SendAckNack(PROTOCOL_TYPE_RESET, PROTOCOL_STATUS_ERROR);
        return;
    }
    uint32_t magic;
    memcpy(&magic, payload, sizeof(magic));
    if (magic != PROTOCOL_RESET_MAGIC) {
        SendAckNack(PROTOCOL_TYPE_RESET, PROTOCOL_STATUS_ERROR);
        return;
    }
    HAL_NVIC_SystemReset();
}

static void HandleFrame(const Protocol_RxParser_t *parser)
{
    uint8_t type = parser->header.type;
    const uint8_t *payload = Protocol_RxParserPayload(parser);
    uint16_t len = parser->header.length;

    if (parser->header.version != PROTOCOL_VERSION) {
        SendAckNack(type, PROTOCOL_STATUS_ERROR);
        return;
    }

    switch (type) {
        case PROTOCOL_TYPE_TELEMETRY:
            /* length=0 — запрос "по требованию", отвечаем одним пакетом.
             * Приходить может в любом режиме, включая периодические. */
            if (len == 0) {
                SendTelemetryOnce();
            } else {
                SendAckNack(type, PROTOCOL_STATUS_ERROR);
            }
            break;
        case PROTOCOL_TYPE_MODE_CONFIG:
            HandleModeConfig(payload, len);
            break;
        case PROTOCOL_TYPE_RESET:
            HandleReset(payload, len);
            break;
        case PROTOCOL_TYPE_TIME_SYNC:
        case PROTOCOL_TYPE_CALIB_ACCEL:
        case PROTOCOL_TYPE_CALIB_GYRO:
            HandleUnimplemented(type);
            break;
        default:
            /* TELEMETRY-данные, ACK и ASCII — это наши же исходящие типы;
             * хост не должен слать их сюда. Неизвестный тип — тоже сюда. */
            SendAckNack(type, PROTOCOL_STATUS_ERROR);
            break;
    }
}

/* Разбирает накопленные ISR байты. Вызывается из тела задачи, поэтому
 * отправка ответов идёт тем же блокирующим путём, что и телеметрия. */
static void ProcessRxBytes(void)
{
    while (s_rxTail != s_rxHead) {
        uint8_t byte = s_rxBuf[s_rxTail];
        uint32_t receivedMs = s_rxTimestamp[s_rxTail];
        s_rxTail = (uint16_t)((s_rxTail + 1u) & TASK_UART_RX_BUF_MASK);

        if (Protocol_RxParserFeedTimed(&s_rxParser, byte, receivedMs) == PROTOCOL_RX_FRAME_OK) {
            HandleFrame(&s_rxParser);
        }
    }
}

static void vTask_UART_BodyFunction(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    TickType_t lastTelemetryTime = xLastWakeTime;
    uint8_t mode = s_mode;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_UART_RX_POLL_MS));
        ProcessRxBytes();

        /* Re-read after commands/ACK: never send using the old mode.
         * Start the new period after acceptance; no catch-up burst. */
        TickType_t now = xTaskGetTickCount();
        uint8_t requestedMode = s_mode;
        if (requestedMode != mode) {
            mode = requestedMode;
            lastTelemetryTime = now;
            continue;
        }

        /* В ON_REQUEST самостоятельная отправка запрещена — телеметрию
         * запрашивают кадром PROTOCOL_TYPE_TELEMETRY с length=0. */
        uint16_t periodMs;
        if (!Protocol_TelemetryPeriodMs(mode, &periodMs)) continue;
        if ((TickType_t)(now - lastTelemetryTime) < pdMS_TO_TICKS(periodMs)) continue;

        SendTelemetryOnce();
        lastTelemetryTime = now;
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
        return;
    }

    /* Приём включается после создания задачи: до этого байты хоста просто
     * не читаются, а флаг RXNE сбрасывается первым же обращением к DR.
     * Приоритет 5 — порога configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
     * ниже, чем у задач, и совместим с FreeRTOS (прерывание не трогает её API). */
    s_rxHead = 0;
    s_rxTail = 0;
    s_rxOverflow = 0;
    Protocol_RxParserReset(&s_rxParser);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void Task_UART_Stop(void)
{
    if (xTask_UARTHandle != NULL) {
        vTaskDelete(xTask_UARTHandle);
        xTask_UARTHandle = NULL;
    }
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
}

bool Task_UART_IsRunning(void)
{
    if (xTask_UARTHandle == NULL)
        return false;
    return (eTaskGetState(xTask_UARTHandle) != eDeleted);
}

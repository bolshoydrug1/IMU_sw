/* Runs the real UART ISR, command dispatcher and task loop with mocked time. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

static jmp_buf s_testDone;
static unsigned s_testCycles, s_testLimit;
static uint8_t s_testRequestedMode;
static bool s_testInjectMode;

static TickType_t xTaskGetTickCount(void) { return g_tickMs; }

static void vTaskDelayUntil(TickType_t *last, TickType_t step)
{
    if (++s_testCycles > s_testLimit) longjmp(s_testDone, 1);
    assert(step == TASK_UART_RX_POLL_MS);
    *last += step;
    g_tickMs = *last;
    if (s_testInjectMode && s_testCycles == 1) {
        uint8_t frame[9]; uint16_t len;
        assert(Protocol_BuildFrame(frame, sizeof(frame), PROTOCOL_TYPE_MODE_CONFIG,
                                  &s_testRequestedMode, 1, &len));
        Mock_RxBytes(frame, len);
    }
}

static void ResetUart(void)
{
    Mock_Reset();
    s_rxHead = s_rxTail = 0;
    s_rxOverflow = 0;
    Protocol_RxParserReset(&s_rxParser);
    s_mode = TASK_UART_DEFAULT_MODE;
}

static void RunTask(unsigned cycles)
{
    s_testLimit = cycles;
    s_testCycles = 0;
    if (setjmp(s_testDone) == 0) vTask_UART_BodyFunction(NULL);
}

static void CheckSchedules(void)
{
    const uint8_t modes[] = {PROTOCOL_MODE_100HZ, PROTOCOL_MODE_250HZ,
                            PROTOCOL_MODE_500HZ, PROTOCOL_MODE_ON_REQUEST};
    /* Every transition, including same-mode ACKs. */
    for (unsigned old = 0; old < 4; ++old) {
        for (unsigned next = 0; next < 4; ++next) {
            ResetUart();
            s_mode = modes[old];
            s_testRequestedMode = modes[next];
            s_testInjectMode = true;
            uint32_t start = g_tickMs;
            RunTask(11);
            assert(s_mode == modes[next]);
            assert(g_txCount >= 1 && g_txTypes[0] == PROTOCOL_TYPE_ACK_NACK);
            uint16_t period;
            if (!Protocol_TelemetryPeriodMs(modes[next], &period)) {
                assert(g_txCount == 1); /* ACK only, no trailing telemetry. */
            } else {
                uint32_t anchor = start + (old == next ? 0 : TASK_UART_RX_POLL_MS);
                assert(g_txCount == 1 + (g_tickMs - anchor) / period);
                for (unsigned i = 1; i < g_txCount; ++i) {
                    assert(g_txTypes[i] == PROTOCOL_TYPE_TELEMETRY);
                    assert(g_txTimes[i] == anchor + i * period);
                }
            }
            assert(g_txBuf[5] == PROTOCOL_TYPE_MODE_CONFIG && g_txBuf[6] == PROTOCOL_STATUS_OK);
        }
    }
    s_testInjectMode = false;
    ResetUart();
    g_tickMs = UINT32_MAX - 3;
    s_mode = PROTOCOL_MODE_250HZ;
    RunTask(4);
    assert(g_txCount == 2 && g_txTimes[0] == 0 && g_txTimes[1] == 4);
}

static void CheckCommands(void)
{
    for (uint8_t mode = 0; mode <= 5; ++mode) {
        ResetUart();
        Mock_SendCommand(PROTOCOL_TYPE_MODE_CONFIG, &mode, 1);
        bool supported = mode == 0 || mode == 1 || mode == 2 || mode == 4;
        assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG,
                         supported ? PROTOCOL_STATUS_OK : PROTOCOL_STATUS_ERROR));
        assert(s_mode == (supported ? mode : TASK_UART_DEFAULT_MODE));
    }
    ResetUart();
    Mock_SendCommand(PROTOCOL_TYPE_MODE_CONFIG, NULL, 0);
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_ERROR));
    ResetUart();
    uint8_t extra[2] = {1, 2};
    Mock_SendCommand(PROTOCOL_TYPE_MODE_CONFIG, extra, 2);
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_ERROR));
    ResetUart();
    s_mode = PROTOCOL_MODE_ON_REQUEST;
    Mock_SendCommand(PROTOCOL_TYPE_TELEMETRY, NULL, 0);
    assert(Mock_TelemetryIs(PROTOCOL_TELEMETRY_STATUS_ERROR));
    ResetUart();
    Mock_SendCommand(PROTOCOL_TYPE_TIME_SYNC, NULL, 0);
    assert(Mock_AckIs(PROTOCOL_TYPE_TIME_SYNC, PROTOCOL_STATUS_ERROR));

    uint8_t frame[9], mode = PROTOCOL_MODE_250HZ;
    uint16_t len;
    ResetUart();
    assert(Protocol_BuildFrame(frame, sizeof(frame), PROTOCOL_TYPE_MODE_CONFIG, &mode, 1, &len));
    frame[6] ^= 1; /* Corrupt CRC must not change the mode or produce ACK. */
    Mock_SendRaw(frame, len);
    assert(g_txCount == 0 && s_mode == TASK_UART_DEFAULT_MODE);
    Mock_SendCommand(PROTOCOL_TYPE_MODE_CONFIG, &mode, 1);
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_OK));

    ResetUart();
    assert(Protocol_BuildFrame(frame, sizeof(frame), PROTOCOL_TYPE_MODE_CONFIG, &mode, 1, &len));
    frame[2] = (uint8_t)(PROTOCOL_VERSION - 1);
    uint16_t crc = Protocol_ComputeCRC16(frame + 1, 5);
    frame[6] = (uint8_t)crc; frame[7] = (uint8_t)(crc >> 8);
    Mock_SendRaw(frame, len);
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_ERROR));
    assert(s_mode == TASK_UART_DEFAULT_MODE);
}

static void CheckParser(void)
{
    uint8_t frame[9], mode = PROTOCOL_MODE_250HZ;
    uint16_t len;
    assert(Protocol_BuildFrame(frame, sizeof(frame), PROTOCOL_TYPE_MODE_CONFIG,
                              &mode, 1, &len));
    ResetUart();
    Mock_SendRaw(frame, 5); /* Truncated header, then next command after silence. */
    g_tickMs += PROTOCOL_RX_FRAME_TIMEOUT_MS + 1;
    Mock_SendRaw(frame, len);
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_OK));

    ResetUart();
    Mock_SendRaw(frame, 5);
    Mock_SendRaw(frame, len); /* No silence: recover next header after bad CRC. */
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_OK));

    ResetUart();
    /* Both bursts are only 1 ms apart but task processes them much later. */
    Mock_RxBytes(frame, 4);
    g_tickMs += 1;
    Mock_RxBytes(frame + 4, len - 4);
    g_tickMs += 100;
    ProcessRxBytes();
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_OK));

    ResetUart();
    /* Gap captured in ISR must be detected even if both bursts are queued. */
    Mock_RxBytes(frame, 5);
    g_tickMs += PROTOCOL_RX_FRAME_TIMEOUT_MS + 1;
    Mock_RxBytes(frame, len);
    ProcessRxBytes();
    assert(Mock_AckIs(PROTOCOL_TYPE_MODE_CONFIG, PROTOCOL_STATUS_OK));

    Protocol_RxParser_t parser = {0};
    for (unsigned i = 0; i < 5; ++i)
        Protocol_RxParserFeedTimed(&parser, frame[i], UINT32_MAX - 5);
    unsigned accepted = 0;
    for (unsigned i = 5; i < len; ++i)
        if (Protocol_RxParserFeedTimed(&parser, frame[i], 2) == PROTOCOL_RX_FRAME_OK) ++accepted;
    assert(accepted == 1); /* Small inter-byte gap across tick wrap. */

    uint8_t huge[] = {0xAA, 7, PROTOCOL_VERSION, 0xFF, 0xFF};
    Protocol_RxParserReset(&parser);
    Protocol_RxResult_t result = PROTOCOL_RX_NEED_MORE;
    for (unsigned i = 0; i < sizeof(huge); ++i) result = Protocol_RxParserFeed(&parser, huge[i]);
    assert(result == PROTOCOL_RX_OVERFLOW && parser.len == 0);
    assert(!Protocol_ParseFrame(huge, sizeof(huge), NULL, NULL));
    uint8_t wrapped[8] = {0xAA, 7, PROTOCOL_VERSION, 0xFF, 0xFF};
    assert(!Protocol_ParseFrame(wrapped, sizeof(wrapped), NULL, NULL));
    assert(!Protocol_BuildFrame(wrapped, sizeof(wrapped), 7, NULL, UINT16_MAX, NULL));

    uint8_t payload[PROTOCOL_RX_MAX_PAYLOAD], large[PROTOCOL_OVERHEAD_BYTES + PROTOCOL_RX_MAX_PAYLOAD];
    memset(payload, PROTOCOL_HEAD, sizeof(payload));
    assert(Protocol_BuildFrame(large, sizeof(large), PROTOCOL_TYPE_ASCII_MSG,
                              payload, sizeof(payload), &len));
    Protocol_RxParserReset(&parser);
    for (unsigned i = 0; i < len; ++i) result = Protocol_RxParserFeed(&parser, large[i]);
    assert(result == PROTOCOL_RX_FRAME_OK && parser.header.length == sizeof(payload));
    assert(memcmp(Protocol_RxParserPayload(&parser), payload, sizeof(payload)) == 0);
}

int main(void)
{
    CheckCommands();
    CheckSchedules();
    CheckParser();
    puts("UART commands, 16 mode transitions, RX timeout and resync: all assertions passed");
    return 0;
}

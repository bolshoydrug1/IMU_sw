#include "Protocol_IMU.h"
#include <string.h>

_Static_assert(sizeof(Protocol_TelemetryData_t) == 49, "Telemetry wire size changed");
_Static_assert(offsetof(Protocol_TelemetryData_t, temperature_c) == 44, "Telemetry temperature offset changed");
_Static_assert(offsetof(Protocol_TelemetryData_t, status) == 48, "Telemetry status offset changed");

/* CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, без рефлексии на входе/выходе. */
uint16_t Protocol_ComputeCRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

bool Protocol_BuildFrame(uint8_t *outBuf, uint16_t outBufCap, uint8_t type,
                          const uint8_t *payload, uint16_t payloadLen,
                          uint16_t *outFrameLen)
{
    uint32_t frameLen = (uint32_t)PROTOCOL_OVERHEAD_BYTES + payloadLen;
    if (frameLen > outBufCap) {
        return false;
    }

    outBuf[0] = PROTOCOL_HEAD;
    outBuf[1] = type;
    outBuf[2] = PROTOCOL_VERSION;
    outBuf[3] = (uint8_t)(payloadLen & 0xFF);
    outBuf[4] = (uint8_t)((payloadLen >> 8) & 0xFF);

    if (payloadLen > 0 && payload != NULL) {
        memcpy(&outBuf[5], payload, payloadLen);
    }

    /* CRC считается по type+version+length+payload, без head. */
    uint16_t crc = Protocol_ComputeCRC16(&outBuf[1], (uint16_t)(4 + payloadLen));
    outBuf[5 + payloadLen]     = (uint8_t)(crc & 0xFF);
    outBuf[5 + payloadLen + 1] = (uint8_t)((crc >> 8) & 0xFF);
    outBuf[5 + payloadLen + 2] = PROTOCOL_TAIL;

    if (outFrameLen) {
        *outFrameLen = (uint16_t)frameLen;
    }

    return true;
}

bool Protocol_ParseFrame(const uint8_t *buf, uint16_t bufLen,
                          Protocol_Header_t *outHeader,
                          const uint8_t **outPayload)
{
    if (bufLen < PROTOCOL_OVERHEAD_BYTES) {
        return false;
    }
    if (buf[0] != PROTOCOL_HEAD) {
        return false;
    }

    uint8_t type = buf[1];
    uint8_t version = buf[2];
    uint16_t length = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);

    uint32_t expectedFrameLen = (uint32_t)PROTOCOL_OVERHEAD_BYTES + length;
    if (bufLen < expectedFrameLen) {
        return false; /* кадр ещё не пришёл целиком */
    }

    uint16_t crcReceived = (uint16_t)buf[5 + length] | ((uint16_t)buf[5 + length + 1] << 8);
    uint16_t crcComputed = Protocol_ComputeCRC16(&buf[1], (uint16_t)(4 + length));
    if (crcReceived != crcComputed) {
        return false;
    }

    if (buf[5 + length + 2] != PROTOCOL_TAIL) {
        return false;
    }

    if (outHeader) {
        outHeader->head = buf[0];
        outHeader->type = type;
        outHeader->version = version;
        outHeader->length = length;
    }
    if (outPayload) {
        *outPayload = (length > 0) ? &buf[5] : NULL;
    }

    return true;
}

bool Protocol_ModeIsSupported(uint8_t mode)
{
    switch (mode) {
        case PROTOCOL_MODE_100HZ:
        case PROTOCOL_MODE_250HZ:
        case PROTOCOL_MODE_500HZ:
        case PROTOCOL_MODE_ON_REQUEST:
            return true;
        default:
            return false;
    }
}

bool Protocol_TelemetryPeriodMs(uint8_t mode, uint16_t *outPeriodMs)
{
    uint16_t period;
    switch (mode) {
        case PROTOCOL_MODE_100HZ:  period = 10; break;
        case PROTOCOL_MODE_250HZ:  period = 4;  break;
        case PROTOCOL_MODE_500HZ:  period = 2;  break;
        default: return false; /* ON_REQUEST и неизвестные — периода нет */
    }
    if (outPeriodMs) {
        *outPeriodMs = period;
    }
    return true;
}

void Protocol_RxParserReset(Protocol_RxParser_t *parser)
{
    parser->len = 0;
    parser->last_byte_ms = 0;
}

/* Retain a possible next header after a bad candidate. Never restart on 0xAA
 * inside a still-valid candidate: payload bytes may contain any value. */
static void RxParserResync(Protocol_RxParser_t *parser)
{
    uint16_t offset = 1;
    while (offset < parser->len && parser->buf[offset] != PROTOCOL_HEAD) ++offset;
    parser->len = (uint16_t)(parser->len - offset);
    memmove(parser->buf, parser->buf + offset, parser->len);
}

Protocol_RxResult_t Protocol_RxParserFeed(Protocol_RxParser_t *parser, uint8_t byte)
{
    /* Поиск начала кадра: всё, что не 0xAA, до этого момента — шум. */
    if (parser->len == 0) {
        if (byte != PROTOCOL_HEAD) {
            return PROTOCOL_RX_NEED_MORE;
        }
        parser->buf[0] = byte;
        parser->len = 1;
        return PROTOCOL_RX_NEED_MORE;
    }

    parser->buf[parser->len++] = byte;

    Protocol_RxResult_t result = PROTOCOL_RX_NEED_MORE;
    while (parser->len >= sizeof(Protocol_Header_t)) {
        uint16_t payloadLen = (uint16_t)parser->buf[3] | ((uint16_t)parser->buf[4] << 8);
        /* Check payload before addition, including lengths near UINT16_MAX. */
        if (payloadLen > PROTOCOL_RX_MAX_PAYLOAD) {
            result = PROTOCOL_RX_OVERFLOW;
            RxParserResync(parser);
            continue;
        }
        uint16_t frameLen = (uint16_t)(PROTOCOL_OVERHEAD_BYTES + payloadLen);
        if (parser->len < frameLen) break;

        Protocol_Header_t header;
        if (Protocol_ParseFrame(parser->buf, frameLen, &header, NULL)) {
            Protocol_RxParserReset(parser);
            parser->header = header;
            return PROTOCOL_RX_FRAME_OK;
        }
        result = PROTOCOL_RX_FRAME_BAD;
        RxParserResync(parser);
    }
    return result;
}

Protocol_RxResult_t Protocol_RxParserFeedTimed(Protocol_RxParser_t *parser,
                                             uint8_t byte, uint32_t receivedMs)
{
    if (parser->len != 0 &&
        (uint32_t)(receivedMs - parser->last_byte_ms) > PROTOCOL_RX_FRAME_TIMEOUT_MS)
        Protocol_RxParserReset(parser);
    Protocol_RxResult_t result = Protocol_RxParserFeed(parser, byte);
    parser->last_byte_ms = receivedMs;
    return result;
}

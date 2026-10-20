#include "Protocol_IMU.h"
#include <string.h>

_Static_assert(sizeof(Protocol_TelemetryData_t) == 45, "Telemetry wire size changed");
_Static_assert(offsetof(Protocol_TelemetryData_t, status) == 44, "Telemetry status offset changed");

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
    uint16_t frameLen = (uint16_t)(PROTOCOL_OVERHEAD_BYTES + payloadLen);
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
        *outFrameLen = frameLen;
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

    uint16_t expectedFrameLen = (uint16_t)(PROTOCOL_OVERHEAD_BYTES + length);
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

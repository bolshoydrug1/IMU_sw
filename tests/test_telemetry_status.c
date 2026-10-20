/* Prepended with the real FusionOutput_t and PrepareTelemetry by the runner. */
#include <assert.h>
#include <stdio.h>

static void CheckError(const FusionOutput_t *out, uint64_t now)
{
    Protocol_TelemetryData_t data;
    memset(&data, 0xFF, sizeof(data));
    PrepareTelemetry(out, now, &data);
    assert(data.timestamp_ms == now && data.status == PROTOCOL_TELEMETRY_STATUS_ERROR);
    const uint8_t *bytes = (const uint8_t *)&data;
    for (int i = 8; i < 44; ++i) assert(bytes[i] == 0);
}

int main(void)
{
    FusionOutput_t out = {0};
    CheckError(NULL, 100);
    CheckError(&out, 100);
    out.valid = true;
    out.timestamp_ms = 100;
    out.roll_deg = 25;
    out.gyro_dps[1] = -5;
    out.accel_g[2] = 1;
    CheckError(&out, 111);
    Protocol_TelemetryData_t data;
    PrepareTelemetry(&out, 110, &data);
    assert(data.status == PROTOCOL_TELEMETRY_STATUS_OK && data.timestamp_ms == 110);
    assert(data.roll_deg == 25 && data.gyro_y_dps == -5 && data.accel_z_g == 1);
    out.roll_deg = NAN;
    CheckError(&out, 100);
    out.roll_deg = 25;
    out.gyro_dps[0] = INFINITY;
    CheckError(&out, 100);
    out.gyro_dps[0] = 0;
    out.timestamp_ms = UINT32_MAX - 1;
    PrepareTelemetry(&out, UINT64_C(0x100000000), &data);
    assert(data.status == PROTOCOL_TELEMETRY_STATUS_OK);

    uint8_t frame[53];
    uint16_t length;
    assert(Protocol_BuildFrame(frame, sizeof(frame), PROTOCOL_TYPE_TELEMETRY,
                              (const uint8_t *)&data, sizeof(data), &length));
    assert(length == 53 && frame[2] == 2 && frame[3] == 45 && frame[4] == 0);
    assert(frame[49] == PROTOCOL_TELEMETRY_STATUS_OK && frame[52] == PROTOCOL_TAIL);
    Protocol_Header_t header;
    const uint8_t *payload;
    assert(Protocol_ParseFrame(frame, length, &header, &payload));
    assert(header.length == 45 && memcmp(payload, &data, 45) == 0);
    PrepareTelemetry(NULL, 200, &data);
    assert(Protocol_BuildFrame(frame, sizeof(frame), PROTOCOL_TYPE_TELEMETRY,
                              (const uint8_t *)&data, sizeof(data), &length));
    assert(frame[49] == PROTOCOL_TELEMETRY_STATUS_ERROR);
    assert(Protocol_ParseFrame(frame, length, &header, &payload));
    frame[49] ^= 1;
    assert(!Protocol_ParseFrame(frame, length, &header, &payload));
    puts("Telemetry status and wire format: all assertions passed");
    return 0;
}

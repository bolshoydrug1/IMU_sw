#ifndef PROTOCOL_PROTOCOL_IMU_H_
#define PROTOCOL_PROTOCOL_IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PROTOCOL_VERSION   0x01
#define PROTOCOL_HEAD      0xAA
#define PROTOCOL_TAIL      0x55

/* head(1)+type(1)+version(1)+length(2)+crc16(2)+tail(1), без учёта payload'а. */
#define PROTOCOL_OVERHEAD_BYTES 8

#define PROTOCOL_ASCII_MAX_LEN   64
#define PROTOCOL_RESET_MAGIC     0xDEADBEEFu

typedef enum {
    PROTOCOL_TYPE_TELEMETRY   = 0x01, /* device<->host: запрос (length=0) / данные */
    PROTOCOL_TYPE_ACK_NACK    = 0x02, /* device->host: подтверждение команды */
    PROTOCOL_TYPE_TIME_SYNC   = 0x03, /* host->device запрос, device->host ответ */
    PROTOCOL_TYPE_CALIB_ACCEL = 0x04, /* host->device запрос, device->host ответ (Protocol_CalibResponse_t) */
    PROTOCOL_TYPE_CALIB_GYRO  = 0x05, /* аналогично CALIB_ACCEL */
    PROTOCOL_TYPE_ASCII_MSG   = 0x06, /* device->host: лог/статус в виде текста */
    PROTOCOL_TYPE_MODE_CONFIG = 0x07, /* host->device */
    PROTOCOL_TYPE_RESET       = 0x08, /* host->device */
} Protocol_Type_t;

typedef enum {
    PROTOCOL_STATUS_OK    = 0x00,
    PROTOCOL_STATUS_ERROR = 0x01,
} Protocol_Status_t;

typedef enum {
    PROTOCOL_MODE_100HZ      = 0x00,
    PROTOCOL_MODE_250HZ      = 0x01,
    PROTOCOL_MODE_500HZ      = 0x02,
    PROTOCOL_MODE_1000HZ     = 0x03,
    PROTOCOL_MODE_ON_REQUEST = 0x04, /* не слать самостоятельно, только отвечать на TELEMETRY-запрос */
} Protocol_Mode_t;

/* Фиксированная часть кадра перед payload'ом (5 байт). CRC и tail идут
 * после payload переменной длины, поэтому не входят в этот struct — см.
 * Protocol_BuildFrame/Protocol_ParseFrame. */
#pragma pack(push, 1)
typedef struct {
    uint8_t  head;
    uint8_t  type;
    uint8_t  version;
    uint16_t length; /* длина payload'а в байтах */
} Protocol_Header_t;
#pragma pack(pop)

/* ---- Payload-структуры команд ---- */

#pragma pack(push, 1)

/* 0x01, ответ (device->host). Запрос — тот же тип, length=0, без структуры.
 * gyro/accel/roll/pitch/yaw — не сырые показания одного датчика, а результат
 * голосования между BMI270/LSM6DS3TR/MPU6500 (см. Task_Filter): усреднение по
 * тем сенсорам, что не отбракованы как расходящиеся с остальными. */
typedef struct {
    uint64_t timestamp_ms;  /* метка времени сэмпла: мс с включения устройства (после команды TIME_SYNC — выровнена на часы хоста) */

    float    gyro_x_dps;    /* угловая скорость вокруг оси X, градусы/с (dps) */
    float    gyro_y_dps;    /* угловая скорость вокруг оси Y, градусы/с (dps) */
    float    gyro_z_dps;    /* угловая скорость вокруг оси Z, градусы/с (dps) */

    float    accel_x_g;     /* линейное ускорение по оси X, в единицах g (9.81 м/с^2) */
    float    accel_y_g;     /* линейное ускорение по оси Y, в единицах g */
    float    accel_z_g;     /* линейное ускорение по оси Z, в единицах g */

    float    roll_deg;      /* крен — поворот вокруг продольной оси X, градусы */
    float    pitch_deg;     /* тангаж — поворот вокруг поперечной оси Y, градусы */
    float    yaw_deg;       /* рыскание — поворот вокруг вертикальной оси Z, градусы
                              * (без магнитометра — это интегрированный курс от
                              * гироскопа, подвержен дрейфу, не абсолютный азимут) */

} Protocol_TelemetryData_t; /* 44 байта: 8 (timestamp) + 4*9 (float-поля) */

/* 0x02 */
typedef struct {
    uint8_t ack_type; /* эхо Protocol_Type_t команды, к которой относится ответ */
    uint8_t status;   /* Protocol_Status_t */
} Protocol_AckNack_t;

/* 0x03, запрос host->device */
typedef struct {
    uint64_t host_time_ms; /* эталонное время хоста; device сохраняет смещение
                             * и с этого момента метки времени телеметрии
                             * выравниваются на часы хоста */
} Protocol_TimeSyncRequest_t;

/* 0x03, ответ device->host */
typedef struct {
    uint64_t device_tick_ms; /* тик устройства на момент обработки запроса,
                               * для расчёта round-trip задержки на хосте */
} Protocol_TimeSyncResponse_t;

/* 0x04/0x05, запрос — пустой payload (length=0), без структуры. Триггер
 * фиксированной процедуры калибровки на устройстве. */

/* 0x04/0x05, ответ device->host (тип пакета совпадает с типом запроса) */
typedef struct {
    uint8_t status; /* Protocol_Status_t */
    float   bias_x;
    float   bias_y;
    float   bias_z;
} Protocol_CalibResponse_t;

/* 0x06: payload — сырые ASCII-байты длиной length (см. Protocol_Header_t.length),
 * до PROTOCOL_ASCII_MAX_LEN, без структуры и без обязательного null-terminator в кадре. */

/* 0x07, запрос host->device. Ответ — Protocol_AckNack_t. */
typedef struct {
    uint8_t mode; /* Protocol_Mode_t */
} Protocol_ModeConfig_t;

/* 0x08, запрос host->device. Ответа нет — устройство перезагружается. */
typedef struct {
    uint32_t confirm_magic; /* должен точно равняться PROTOCOL_RESET_MAGIC,
                              * иначе команда игнорируется (защита от случайного
                              * сброса из-за битого/шумового пакета) */
} Protocol_ResetRequest_t;

#pragma pack(pop)

/* ---- Общие функции сборки/разбора кадра ---- */

uint16_t Protocol_ComputeCRC16(const uint8_t *data, uint16_t len);

/* Собирает полный кадр (head..tail) в outBuf. Возвращает false, если
 * outBufCap не хватает под PROTOCOL_OVERHEAD_BYTES + payloadLen. */
bool Protocol_BuildFrame(uint8_t *outBuf, uint16_t outBufCap, uint8_t type,
                          const uint8_t *payload, uint16_t payloadLen,
                          uint16_t *outFrameLen);

/* Проверяет head/tail/CRC во входном буфере и возвращает заголовок +
 * указатель на payload внутри buf (без копирования). Возвращает false,
 * если кадр не прошёл валидацию (в т.ч. если в buf ещё не пришли все байты). */
bool Protocol_ParseFrame(const uint8_t *buf, uint16_t bufLen,
                          Protocol_Header_t *outHeader,
                          const uint8_t **outPayload);

#endif /* PROTOCOL_PROTOCOL_IMU_H_ */

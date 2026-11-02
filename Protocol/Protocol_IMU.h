#ifndef PROTOCOL_PROTOCOL_IMU_H_
#define PROTOCOL_PROTOCOL_IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PROTOCOL_VERSION   0x03 /* Telemetry v3 adds filtered temperature in Celsius. */
#define PROTOCOL_HEAD      0xAA
#define PROTOCOL_TAIL      0x55

/* head(1)+type(1)+version(1)+length(2)+crc16(2)+tail(1), без учёта payload'а. */
#define PROTOCOL_OVERHEAD_BYTES 8

#define PROTOCOL_ASCII_MAX_LEN   64
#define PROTOCOL_RESET_MAGIC     0xDEADBEEFu

/* Приём от хоста: заведомо больше любого штатного запроса (MODE_CONFIG — 1
 * байт, TIME_SYNC — 8), но достаточно мало, чтобы отбрасывать заоморфленные
 * кадры с абсурдной длиной до записи в буфер парсера. */
#define PROTOCOL_RX_MAX_PAYLOAD 64u
/* Maximum gap between received bytes of one command (not task polling time). */
#define PROTOCOL_RX_FRAME_TIMEOUT_MS 20u

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

/* Telemetry states are independent of command ACK/NACK statuses.
 * Append new states with explicit, unique codes in 0x02..0xFF;
 * never renumber existing codes. On the wire this enum is stored as uint8_t.
 * Receivers must treat unknown codes as unsupported, not as OK. */
typedef enum {
    PROTOCOL_TELEMETRY_STATUS_OK    = 0x00, /* Measurements are usable. */
    PROTOCOL_TELEMETRY_STATUS_ERROR = 0x01, /* No usable result; measurements are zero. */
} Protocol_TelemetryStatus_t;

typedef enum {
    PROTOCOL_MODE_100HZ      = 0x00,
    PROTOCOL_MODE_250HZ      = 0x01,
    PROTOCOL_MODE_500HZ      = 0x02,
    /* 0x03 (1000Гц) устройством не поддерживается и не переиспользуется:
     * старые пакеты хоста должны отклоняться NACK, а не молча означать
     * другой режим. */
    PROTOCOL_MODE_ON_REQUEST = 0x04, /* не слать самостоятельно, только отвечать на TELEMETRY-запрос */
} Protocol_Mode_t;

/* Поддерживается ли режим устройством (периодический или ON_REQUEST). */
bool Protocol_ModeIsSupported(uint8_t mode);

/* Период самостоятельной отправки телеметрии, мс. Возвращает false для
 * ON_REQUEST (отправка только по запросу) и неподдерживаемых значений. */
bool Protocol_TelemetryPeriodMs(uint8_t mode, uint16_t *outPeriodMs);

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
    uint64_t timestamp_ms;  /* Время подготовки отправки, мс с запуска. */

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

    float temperature_c;   /* Filtered mean of usable sensor temperatures, Celsius. */
    uint8_t status;         /* Protocol_TelemetryStatus_t encoded as one byte.
                            * On error all ten float fields are zero. */
} Protocol_TelemetryData_t; /* v3: 49 bytes, status at payload offset 48. */

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

/* ---- Инкрементальный разбор потока кадров из UART (приём команд) ----
 * Кормится по одному байту; буфер кадра принадлежит парсеру, поэтому разбор
 * не зависит от размера кольцевого буфера UART и тестируется на хосте. */
typedef enum {
    PROTOCOL_RX_NEED_MORE = 0, /* кадр ещё не собран целиком */
    PROTOCOL_RX_FRAME_OK,      /* собран валидный кадр, header заполнен */
    PROTOCOL_RX_FRAME_BAD,     /* кадр собран целиком, но CRC/tail не сошлись */
    PROTOCOL_RX_OVERFLOW,      /* заявленная длина payload не влезает в буфер */
} Protocol_RxResult_t;

typedef struct {
    uint8_t  buf[PROTOCOL_OVERHEAD_BYTES + PROTOCOL_RX_MAX_PAYLOAD];
    uint16_t len;
    uint32_t last_byte_ms; /* Used by Protocol_RxParserFeedTimed. */
    /* Заполняется только вместе с PROTOCOL_RX_FRAME_OK. */
    Protocol_Header_t header;
} Protocol_RxParser_t;

/* Payload последнего кадра; начинается сразу за заголовком в buf парсера.
 * Действителен только до следующего вызова Protocol_RxParserFeed — кадр
 * нужно обработать сразу после получения PROTOCOL_RX_FRAME_OK. */
#define Protocol_RxParserPayload(parser) ((parser)->buf + sizeof(Protocol_Header_t))

void Protocol_RxParserReset(Protocol_RxParser_t *parser);

/* Feeds one byte without timing. Noise before 0xAA is ignored. After a bad
 * CRC/tail/length, retains a possible following header from buffered bytes.
 * After FRAME_OK, payload must be handled before the next call. */
Protocol_RxResult_t Protocol_RxParserFeed(Protocol_RxParser_t *parser, uint8_t byte);

/* Uses byte arrival times, handles tick wrap and discards an incomplete frame
 * after an inter-byte gap greater than PROTOCOL_RX_FRAME_TIMEOUT_MS. */
Protocol_RxResult_t Protocol_RxParserFeedTimed(Protocol_RxParser_t *parser,
                                             uint8_t byte, uint32_t receivedMs);

#endif /* PROTOCOL_PROTOCOL_IMU_H_ */

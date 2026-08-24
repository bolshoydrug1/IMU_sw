#ifndef UART_TASK_UART_TASK_H_
#define UART_TASK_UART_TASK_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

#define TASK_UART_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define TASK_UART_PRIORITY   (tskIDLE_PRIORITY + 2)
#define TASK_UART_NAME       "Task_UART"
#define TASK_UART_PERIOD_MS  2

#define IMU_PACKET_HEAD    0xAA
#define IMU_PACKET_TAIL    0x55
#define IMU_PACKET_TYPE    0x01 /* тип "данные IMU" — задел на другие типы пакетов в будущем */
#define IMU_PROTOCOL_VER   0x01

/* Фиксированная структура телеметрии, без паддинга — размер строго 48 байт.
 * Порядок полей менять нельзя без согласования с парсером на стороне ПК. */
#pragma pack(push, 1)
typedef struct {
    uint8_t  head;          /* всегда IMU_PACKET_HEAD */
    uint8_t  type;          /* всегда IMU_PACKET_TYPE */
    uint8_t  version;       /* всегда IMU_PROTOCOL_VER */
    uint64_t timestamp_ms;  /* миллисекунды с включения устройства, счётчик не переполняется ~49 дней */
    float    gyro_x_dps;
    float    gyro_y_dps;
    float    gyro_z_dps;
    float    accel_x_g;
    float    accel_y_g;
    float    accel_z_g;
    float    roll_deg;
    float    pitch_deg;
    float    yaw_deg;
    uint8_t  tail;          /* всегда IMU_PACKET_TAIL */
} IMU_Packet_t;
#pragma pack(pop)

void Task_UART_Start(void);
void Task_UART_Stop(void);
bool Task_UART_IsRunning(void);

#endif /* UART_TASK_UART_TASK_H_ */

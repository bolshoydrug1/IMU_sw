#ifndef IMU_COMMON_H
#define IMU_COMMON_H

#include <stdint.h>
#include <stdbool.h>

/* Единая структура сэмпла — одинаковая для всех трёх датчиков,
 * передаётся через очереди (см. Task_IMU.h). */
typedef struct {
    float accel_g[3];      /* X,Y,Z, в единицах g */
    float gyro_dps[3];     /* X,Y,Z, в градусах/с */
    float temperature_c;
    bool temperature_valid;
    uint32_t timestamp_ms; /* HAL_GetTick() на момент чтения */
    bool valid;
} IMU_Sample_t;

#endif /* IMU_COMMON_H */

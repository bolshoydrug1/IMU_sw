#ifndef LSM6DS3TR_H
#define LSM6DS3TR_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    float accel_sensitivity_g_per_lsb;
    float gyro_sensitivity_dps_per_lsb;
} LSM6DS3TR_Handle_t;

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
} LSM6DS3TR_RawData_t;

/* TODO: сверить с датащитом именно LSM6DS3TR-C (не обычного LSM6DS3) значение
 * WHO_AM_I и таблицу умолчаний CTRL1_XL/CTRL2_G перед первой прошивкой. */
bool LSM6DS3TR_Init(LSM6DS3TR_Handle_t *dev, SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin);

bool LSM6DS3TR_ReadRaw(LSM6DS3TR_Handle_t *dev, LSM6DS3TR_RawData_t *out);

void LSM6DS3TR_ConvertToPhysical(const LSM6DS3TR_Handle_t *dev,
                                  const LSM6DS3TR_RawData_t *raw,
                                  float accel_g[3], float gyro_dps[3]);

#endif /* LSM6DS3TR_H */

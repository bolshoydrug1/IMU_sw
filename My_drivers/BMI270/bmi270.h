#ifndef BMI270_H
#define BMI270_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    float accel_sensitivity_lsb_per_g;
    float gyro_sensitivity_lsb_per_dps;
} BMI270_Handle_t;

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
} BMI270_RawData_t;

/* TODO: BMI270 требует загрузки init-конфига (~8KB бинарный блоб config_file
 * из бошевской библиотеки) в регистр FEATURES после сброса — без этого
 * акселерометр/гироскоп не переходят в normal mode. Разобраться и добавить
 * загрузку блоба перед первым чтением данных. */
bool BMI270_Init(BMI270_Handle_t *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

bool BMI270_ReadRaw(BMI270_Handle_t *dev, BMI270_RawData_t *out);

void BMI270_ConvertToPhysical(const BMI270_Handle_t *dev,
                               const BMI270_RawData_t *raw,
                               float accel_g[3], float gyro_dps[3]);

#endif /* BMI270_H */

#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    float accel_sensitivity_lsb_per_g;
    float gyro_sensitivity_lsb_per_dps;
} MPU6500_Handle_t;

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;
} MPU6500_RawData_t;

bool MPU6500_Init(MPU6500_Handle_t *dev, SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port, uint16_t cs_pin);

bool MPU6500_ReadRaw(MPU6500_Handle_t *dev, MPU6500_RawData_t *out);
bool MPU6500_DataReady(MPU6500_Handle_t *dev, bool *ready);
float MPU6500_TemperatureC(int16_t raw);

void MPU6500_ConvertToPhysical(const MPU6500_Handle_t *dev,
                                const MPU6500_RawData_t *raw,
                                float accel_g[3], float gyro_dps[3]);

#endif /* MPU6500_H */

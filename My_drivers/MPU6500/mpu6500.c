#include "mpu6500.h"

#define MPU6500_REG_WHO_AM_I     0x75
#define MPU6500_REG_SMPLRT_DIV   0x19
#define MPU6500_REG_CONFIG       0x1A
#define MPU6500_REG_INT_ENABLE   0x38
#define MPU6500_REG_PWR_MGMT_1   0x6B
#define MPU6500_REG_ACCEL_XOUT_H 0x3B /* burst: 6 accel + 2 temp + 6 gyro = 14 байт */

#define MPU6500_SPI_READ_BIT    0x80

static void CS_Low(const MPU6500_Handle_t *dev)  { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static void CS_High(const MPU6500_Handle_t *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

/* SPI-протокол InvenSense: как и у ST, без dummy-байта после адреса. */
static bool MPU6500_ReadRegs(const MPU6500_Handle_t *dev, uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t tx = addr | MPU6500_SPI_READ_BIT;
    bool ok = true;

    CS_Low(dev);
    if (HAL_SPI_Transmit(dev->hspi, &tx, 1, 10) != HAL_OK) ok = false;
    if (ok && HAL_SPI_Receive(dev->hspi, buf, len, 50) != HAL_OK) ok = false;
    CS_High(dev);

    return ok;
}

static bool MPU6500_WriteReg(const MPU6500_Handle_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), value };
    bool ok;

    CS_Low(dev);
    ok = (HAL_SPI_Transmit(dev->hspi, tx, 2, 10) == HAL_OK);
    CS_High(dev);

    return ok;
}

bool MPU6500_Init(MPU6500_Handle_t *dev, SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    /* Значения по умолчанию после сброса: accel FS=+-2g, gyro FS=+-250dps */
    dev->accel_sensitivity_lsb_per_g = 16384.0f;
    dev->gyro_sensitivity_lsb_per_dps = 131.0f;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    /* Выход из sleep mode после POR — обязателен, иначе данные не обновляются. */
    if (!MPU6500_WriteReg(dev, MPU6500_REG_PWR_MGMT_1, 0x00)) {
        return false;
    }

    uint8_t who_am_i = 0;
    if (!MPU6500_ReadRegs(dev, MPU6500_REG_WHO_AM_I, &who_am_i, 1)) {
        return false;
    }
    if (who_am_i != 0x70) return false;
    /* Clear DATA_RDY only by reading INT_STATUS, not by WHO_AM_I reads. */
    if (!MPU6500_WriteReg(dev, 0x37, 0x00)) return false;
    /* Make conversion scales explicit, including after a debugger reset. */
    if (!MPU6500_WriteReg(dev, 0x1B, 0x00) || !MPU6500_WriteReg(dev, 0x1C, 0x00))
        return false;

    /* CONFIG: DLPF_CFG=3 (~41 Гц полоса, внутренний Fs=1кГц) — без этого DLPF
     * выключен и внутренняя частота гироскопа 8кГц, что заливает EXTI/шину. */
    if (!MPU6500_WriteReg(dev, MPU6500_REG_CONFIG, 0x03)) return false;

    /* SMPLRT_DIV=0 -> ODR = 1кГц/(1+0) = 1000 Гц — с запасом выше 500Гц опроса. */
    if (!MPU6500_WriteReg(dev, MPU6500_REG_SMPLRT_DIV, 0x00)) return false;

    /* INT_ENABLE: DATA_RDY_EN=1 — без этого MPU_INT никогда не переключится,
     * даже когда данные реально обновляются на заданном ODR. */
    if (!MPU6500_WriteReg(dev, MPU6500_REG_INT_ENABLE, 0x01)) return false;

    /* TODO: явно настроить ACCEL_CONFIG/GYRO_CONFIG, если диапазон ±2g/±250dps
     * (текущие дефолты после сброса) окажется недостаточным. */

    return true;
}

bool MPU6500_ReadRaw(MPU6500_Handle_t *dev, MPU6500_RawData_t *out)
{
    uint8_t buf[14];

    if (!MPU6500_ReadRegs(dev, MPU6500_REG_ACCEL_XOUT_H, buf, 14)) {
        return false;
    }

    out->accel_raw[0] = (int16_t)((buf[0] << 8) | buf[1]);
    out->accel_raw[1] = (int16_t)((buf[2] << 8) | buf[3]);
    out->accel_raw[2] = (int16_t)((buf[4] << 8) | buf[5]);
    out->temp_raw     = (int16_t)((buf[6] << 8) | buf[7]);
    out->gyro_raw[0]  = (int16_t)((buf[8] << 8) | buf[9]);
    out->gyro_raw[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_raw[2]  = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

bool MPU6500_DataReady(MPU6500_Handle_t *dev, bool *ready)
{
    uint8_t id = 0, status = 0;
    *ready = false;
    if (!MPU6500_ReadRegs(dev, MPU6500_REG_WHO_AM_I, &id, 1) || id != 0x70)
        return false;
    if (!MPU6500_ReadRegs(dev, 0x3A, &status, 1)) return false;
    *ready = (status & 0x01) != 0;
    return true;
}

float MPU6500_TemperatureC(int16_t raw)
{
    return 21.0f + (float)raw / 333.87f;
}

void MPU6500_ConvertToPhysical(const MPU6500_Handle_t *dev,
                                const MPU6500_RawData_t *raw,
                                float accel_g[3], float gyro_dps[3])
{
    for (int i = 0; i < 3; i++) {
        accel_g[i] = (float)raw->accel_raw[i] / dev->accel_sensitivity_lsb_per_g;
        gyro_dps[i] = (float)raw->gyro_raw[i] / dev->gyro_sensitivity_lsb_per_dps;
    }
}

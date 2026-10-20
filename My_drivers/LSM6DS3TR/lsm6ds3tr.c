#include "lsm6ds3tr.h"

#define LSM6DS3TR_REG_WHO_AM_I   0x0F
#define LSM6DS3TR_REG_INT1_CTRL  0x0D
#define LSM6DS3TR_REG_CTRL1_XL   0x10
#define LSM6DS3TR_REG_CTRL2_G    0x11
#define LSM6DS3TR_REG_CTRL3_C    0x12
#define LSM6DS3TR_REG_OUTX_L_G   0x22 /* блок гироскопа: 6 байт, X/Y/Z */
#define LSM6DS3TR_REG_OUTX_L_XL  0x28 /* блок акселерометра: 6 байт, X/Y/Z */

#define LSM6DS3TR_SPI_READ_BIT  0x80

static void CS_Low(const LSM6DS3TR_Handle_t *dev)  { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static void CS_High(const LSM6DS3TR_Handle_t *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

/* SPI-протокол ST: адрес с битом READ, сразу за ним данные без dummy-байта
 * (в отличие от Bosch BMI270) — не перепутать при портировании кода. */
static bool LSM6DS3TR_ReadRegs(const LSM6DS3TR_Handle_t *dev, uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t tx = addr | LSM6DS3TR_SPI_READ_BIT;
    bool ok = true;

    CS_Low(dev);
    if (HAL_SPI_Transmit(dev->hspi, &tx, 1, 10) != HAL_OK) ok = false;
    if (ok && HAL_SPI_Receive(dev->hspi, buf, len, 50) != HAL_OK) ok = false;
    CS_High(dev);

    return ok;
}

static bool LSM6DS3TR_WriteReg(const LSM6DS3TR_Handle_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), value };
    bool ok;

    CS_Low(dev);
    ok = (HAL_SPI_Transmit(dev->hspi, tx, 2, 10) == HAL_OK);
    CS_High(dev);

    return ok;
}

/* Проверяет, что записанное значение реально закрепилось в регистре —
 * "HAL_OK" от Transmit значит только то, что мастер откатал биты по линии,
 * а не то, что ведомый их принял (несовпадение SPI-режима/проводки даёт
 * именно такую картину: HAL молчаливо "успешен", датчик молчаливо игнорирует). */
static bool LSM6DS3TR_WriteRegVerified(const LSM6DS3TR_Handle_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t readback = 0;

    if (!LSM6DS3TR_WriteReg(dev, addr, value)) return false;
    if (!LSM6DS3TR_ReadRegs(dev, addr, &readback, 1)) return false;

    return (readback == value);
}

bool LSM6DS3TR_Init(LSM6DS3TR_Handle_t *dev, SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    /* Значения по умолчанию после сброса: accel FS=+-2g, gyro FS=+-245dps.
     * TODO: пересчитать, если CTRL1_XL/CTRL2_G будут сконфигурированы на
     * другой диапазон (см. README по расчёту потребления/диапазонов). */
    dev->accel_sensitivity_g_per_lsb = 0.000061f;
    dev->gyro_sensitivity_dps_per_lsb = 0.00875f;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    uint8_t who_am_i = 0;
    if (!LSM6DS3TR_ReadRegs(dev, LSM6DS3TR_REG_WHO_AM_I, &who_am_i, 1)) {
        return false;
    }
    if (who_am_i != 0x6A) return false;

    /* CTRL3_C: BDU=1 (не отдавать половину обновлённого сэмпла), IF_INC уже 1 по умолчанию. */
    if (!LSM6DS3TR_WriteRegVerified(dev, LSM6DS3TR_REG_CTRL3_C, 0x44)) return false;

    /* CTRL1_XL: ODR_XL=0111 (833 Гц) — с запасом выше 500Гц опроса Task_IMU,
     * FS_XL=00 (+-2g) -> сенситивность выше не меняется */
    if (!LSM6DS3TR_WriteRegVerified(dev, LSM6DS3TR_REG_CTRL1_XL, 0x70)) return false;

    /* CTRL2_G: ODR_G=0111 (833 Гц), FS_G=00 (+-245dps) */
    if (!LSM6DS3TR_WriteRegVerified(dev, LSM6DS3TR_REG_CTRL2_G, 0x70)) return false;

    /* INT1_CTRL: INT1_DRDY_XL=1, INT1_DRDY_G=1 — не влияет на текущий поллинг,
     * но пусть остаётся консистентно настроенным на случай возврата к EXTI. */
    if (!LSM6DS3TR_WriteRegVerified(dev, LSM6DS3TR_REG_INT1_CTRL, 0x03)) return false;

    return true;
}

bool LSM6DS3TR_ReadRaw(LSM6DS3TR_Handle_t *dev, LSM6DS3TR_RawData_t *out)
{
    uint8_t gyro_buf[6];
    uint8_t accel_buf[6];

    if (!LSM6DS3TR_ReadRegs(dev, LSM6DS3TR_REG_OUTX_L_G, gyro_buf, 6)) {
        return false;
    }
    if (!LSM6DS3TR_ReadRegs(dev, LSM6DS3TR_REG_OUTX_L_XL, accel_buf, 6)) {
        return false;
    }

    out->gyro_raw[0]  = (int16_t)((gyro_buf[1] << 8) | gyro_buf[0]);
    out->gyro_raw[1]  = (int16_t)((gyro_buf[3] << 8) | gyro_buf[2]);
    out->gyro_raw[2]  = (int16_t)((gyro_buf[5] << 8) | gyro_buf[4]);
    out->accel_raw[0] = (int16_t)((accel_buf[1] << 8) | accel_buf[0]);
    out->accel_raw[1] = (int16_t)((accel_buf[3] << 8) | accel_buf[2]);
    out->accel_raw[2] = (int16_t)((accel_buf[5] << 8) | accel_buf[4]);

    return true;
}

bool LSM6DS3TR_DataReady(LSM6DS3TR_Handle_t *dev, bool *ready)
{
    uint8_t id = 0, status = 0;
    *ready = false;
    if (!LSM6DS3TR_ReadRegs(dev, LSM6DS3TR_REG_WHO_AM_I, &id, 1) || id != 0x6A)
        return false;
    if (!LSM6DS3TR_ReadRegs(dev, 0x1E, &status, 1)) return false;
    *ready = (status & 0x03) == 0x03; /* XLDA and GDA */
    return true;
}

void LSM6DS3TR_ConvertToPhysical(const LSM6DS3TR_Handle_t *dev,
                                  const LSM6DS3TR_RawData_t *raw,
                                  float accel_g[3], float gyro_dps[3])
{
    for (int i = 0; i < 3; i++) {
        accel_g[i] = (float)raw->accel_raw[i] * dev->accel_sensitivity_g_per_lsb;
        gyro_dps[i] = (float)raw->gyro_raw[i] * dev->gyro_sensitivity_dps_per_lsb;
    }
}

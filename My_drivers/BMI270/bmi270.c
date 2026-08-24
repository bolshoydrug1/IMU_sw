#include "bmi270.h"

/* NOTE: адреса регистров и порядок байт сверены с общей архитектурой регистровой
 * карты линейки Bosch (BMI160/BMI270 совместимы по расположению ACC/GYR данных).
 * Перед прошивкой на реальном железе перепроверить по актуальному датащиту BMI270. */
#define BMI270_REG_CHIP_ID   0x00
#define BMI270_REG_ACC_DATA  0x0C /* ACC_X_LSB, далее X_MSB,Y_LSB,Y_MSB,Z_LSB,Z_MSB */
#define BMI270_REG_GYR_DATA  0x12 /* GYR_X_LSB, далее аналогично */

#define BMI270_SPI_READ_BIT  0x80

static void CS_Low(const BMI270_Handle_t *dev)  { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static void CS_High(const BMI270_Handle_t *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

/* SPI-чтение регистров Bosch IMU: после байта адреса с битом READ следует один
 * dummy-байт, затем полезные данные — особенность протокола, отличающая от I2C. */
static bool BMI270_ReadRegs(const BMI270_Handle_t *dev, uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t tx = addr | BMI270_SPI_READ_BIT;
    uint8_t dummy;
    bool ok = true;

    CS_Low(dev);
    if (HAL_SPI_Transmit(dev->hspi, &tx, 1, 10) != HAL_OK) ok = false;
    if (ok && HAL_SPI_Receive(dev->hspi, &dummy, 1, 10) != HAL_OK) ok = false;
    if (ok && HAL_SPI_Receive(dev->hspi, buf, len, 50) != HAL_OK) ok = false;
    CS_High(dev);

    return ok;
}

static bool BMI270_WriteReg(const BMI270_Handle_t *dev, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), value };
    bool ok;

    CS_Low(dev);
    ok = (HAL_SPI_Transmit(dev->hspi, tx, 2, 10) == HAL_OK);
    CS_High(dev);

    return ok;
}

bool BMI270_Init(BMI270_Handle_t *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    /* Значения по умолчанию после сброса: accel FS=+-2g, gyro FS=+-2000dps */
    dev->accel_sensitivity_lsb_per_g = 16384.0f;
    dev->gyro_sensitivity_lsb_per_dps = 16.384f;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    uint8_t chip_id = 0;
    if (!BMI270_ReadRegs(dev, BMI270_REG_CHIP_ID, &chip_id, 1)) {
        return false;
    }

    /* TODO: загрузить init-конфиг (config_file, ~8KB) в регистр FEATURES,
     * дождаться internal_status.message == 0x1 (init OK), только после этого
     * переключить ACC/GYR из suspend в normal mode через PWR_CTRL/ACC_CONF.
     * Без этого шага чип отдаёт мусор или замороженные данные. */
    (void)BMI270_WriteReg;

    return true;
}

bool BMI270_ReadRaw(BMI270_Handle_t *dev, BMI270_RawData_t *out)
{
    uint8_t buf[12];

    if (!BMI270_ReadRegs(dev, BMI270_REG_ACC_DATA, buf, 12)) {
        return false;
    }

    out->accel_raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    out->accel_raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    out->accel_raw[2] = (int16_t)((buf[5] << 8) | buf[4]);
    out->gyro_raw[0]  = (int16_t)((buf[7] << 8) | buf[6]);
    out->gyro_raw[1]  = (int16_t)((buf[9] << 8) | buf[8]);
    out->gyro_raw[2]  = (int16_t)((buf[11] << 8) | buf[10]);

    return true;
}

void BMI270_ConvertToPhysical(const BMI270_Handle_t *dev,
                               const BMI270_RawData_t *raw,
                               float accel_g[3], float gyro_dps[3])
{
    for (int i = 0; i < 3; i++) {
        accel_g[i] = (float)raw->accel_raw[i] / dev->accel_sensitivity_lsb_per_g;
        gyro_dps[i] = (float)raw->gyro_raw[i] / dev->gyro_sensitivity_lsb_per_dps;
    }
}

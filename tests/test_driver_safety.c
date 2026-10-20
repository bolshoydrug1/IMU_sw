#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static uint8_t regs[128], address;
static bool io_error;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)spi; (void)timeout;
    if (io_error) return HAL_ERROR;
    address = data[0] & 0x7F;
    if (len == 2) regs[address] = data[1];
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t len, uint32_t timeout)
{
    (void)spi; (void)timeout;
    if (io_error) return HAL_ERROR;
    memcpy(data, regs + address, len);
    return HAL_OK;
}

#if defined(TEST_LSM)
#include "../My_drivers/LSM6DS3TR/lsm6ds3tr.c"
#define DEVICE LSM6DS3TR_Handle_t
#define INIT LSM6DS3TR_Init
#define READY LSM6DS3TR_DataReady
#define ID_REG 0x0F
#define ID 0x6A
#define STATUS_REG 0x1E
#define STATUS_READY 3
#elif defined(TEST_MPU)
#include "../My_drivers/MPU6500/mpu6500.c"
#define DEVICE MPU6500_Handle_t
#define INIT MPU6500_Init
#define READY MPU6500_DataReady
#define ID_REG 0x75
#define ID 0x70
#define STATUS_REG 0x3A
#define STATUS_READY 1
#else
#include "../My_drivers/BMI270/bmi270.c"
#define DEVICE BMI270_Handle_t
#define INIT BMI270_Init
#define ID_REG 0
#define ID 0x24
#endif

int main(void)
{
    DEVICE dev;
    SPI_HandleTypeDef spi = {0};
    GPIO_TypeDef gpio = {0};
    regs[ID_REG] = 0;
    assert(!INIT(&dev, &spi, &gpio, 1));
    regs[ID_REG] = 0xFF;
    assert(!INIT(&dev, &spi, &gpio, 1));
    regs[ID_REG] = ID;
#if defined(TEST_LSM) || defined(TEST_MPU)
    assert(INIT(&dev, &spi, &gpio, 1));
#if defined(TEST_LSM)
    /* One LE burst: temperature + gyro + accel. */
    regs[0x20] = 0; regs[0x21] = 1; /* +256 -> 26 C */
    regs[0x22] = 0xFE; regs[0x23] = 0xFF;
    regs[0x28] = 0x34; regs[0x29] = 0x12;
    LSM6DS3TR_RawData_t raw;
    assert(LSM6DS3TR_ReadRaw(&dev, &raw));
    assert(raw.temp_raw == 256 && raw.gyro_raw[0] == -2 && raw.accel_raw[0] == 0x1234);
    assert(LSM6DS3TR_TemperatureC(raw.temp_raw) == 26);
    assert(LSM6DS3TR_TemperatureC(-256) == 24);
#else
    regs[0x41] = 0xFF; regs[0x42] = 0x9C; /* -100, big endian */
    MPU6500_RawData_t raw;
    assert(MPU6500_ReadRaw(&dev, &raw) && raw.temp_raw == -100);
    assert(MPU6500_TemperatureC(0) == 21);
    assert(fabsf(MPU6500_TemperatureC(-100) - (21 - 100/333.87f)) < 1e-5f);
#endif
    bool ready = true;
    regs[STATUS_REG] = 0;
    assert(READY(&dev, &ready) && !ready);
    regs[STATUS_REG] = STATUS_READY;
    assert(READY(&dev, &ready) && ready);
    regs[ID_REG] = 0xFF;
    assert(!READY(&dev, &ready) && !ready);
    regs[ID_REG] = ID;
    io_error = true;
    assert(!READY(&dev, &ready) && !ready);
#else
    assert(!INIT(&dev, &spi, &gpio, 1)); /* Known ID is not full initialization. */
#endif
    puts("Driver safety: all assertions passed");
    return 0;
}

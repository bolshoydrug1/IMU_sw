#include "stm32f4xx_hal.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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

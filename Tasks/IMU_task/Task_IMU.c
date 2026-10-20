#include "Task_IMU.h"
#include "imu_common.h"
#include "bmi270.h"
#include "lsm6ds3tr.h"
#include "mpu6500.h"
#include "main.h"
#include <math.h>
#include <string.h>

static TaskHandle_t xTask_IMUHandle = NULL;

QueueHandle_t xQueue_BMI270;
QueueHandle_t xQueue_LSM6DS3TR;
QueueHandle_t xQueue_MPU6500;

static BMI270_Handle_t bmi;
static LSM6DS3TR_Handle_t lsm;
static MPU6500_Handle_t mpu;

extern SPI_HandleTypeDef hspi1; /* MPU6500 */
extern SPI_HandleTypeDef hspi2; /* LSM6DS3TR */
extern SPI_HandleTypeDef hspi3; /* BMI270 */

#define SENSOR_RETRY_MS 1000u
#define SENSOR_STALE_MS 100u /* Allows gyro startup; fusion expires samples at 10 ms. */
/* BMI270 is intentionally offline until its full init sequence exists. */
static bool s_lsmOnline, s_mpuOnline;
static uint32_t s_lsmRetry, s_mpuRetry, s_lsmLastData, s_mpuLastData;

/* Оба рабочих датчика сейчас на FS=+-2g (см. LSM6DS3TR_Init/MPU6500_Init),
 * поэтому физически осмысленный модуль accel лежит в (0.1g, 2.2g] — нижняя
 * граница отсекает "мёртвый" датчик, отдающий (0,0,0) (например BMI270 без
 * загруженного init-конфига — SPI-чтение проходит успешно, но данные не
 * обновляются), верхняя — с небольшим запасом под потолок диапазона FS.
 * Без этой проверки "мёртвый" датчик всё равно считается valid (SPI-транзакция
 * ведь реально прошла) и молча портит усреднение в Task_Filter — делит сумму
 * на количество датчиков, включая тот, что дал ровно ноль. */
static bool IsAccelPlausible(const float accel_g[3])
{
    float mag2 = accel_g[0] * accel_g[0] + accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2];
    return (mag2 > 0.01f) && (mag2 < 4.84f); /* |accel| в (0.1g, 2.2g) */
}

static void vTask_IMU_BodyFunction(void *pvParameters)
{
    (void)pvParameters;
    /* Run delays and sensor initialization with the scheduler already active.
     * Creating another FreeRTOS task before this can mask the HAL tick at boot. */
    HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(100));
    (void)BMI270_Init(&bmi, &hspi3, BMI270_CS_GPIO_Port, BMI270_CS_Pin);
    s_lsmOnline = LSM6DS3TR_Init(&lsm, &hspi2, LSM_CS_GPIO_Port, LSM_CS_Pin);
    s_mpuOnline = MPU6500_Init(&mpu, &hspi1, MPU_CS_GPIO_Port, MPU_CS_Pin);
    s_lsmRetry = s_mpuRetry = s_lsmLastData = s_mpuLastData = HAL_GetTick();
    HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_IMU_PERIOD_MS);

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        IMU_Sample_t sample = {0};
        bool ready = false;

        /* Offline sensors are probed at most once per second; no retry loops. */
        if (!s_lsmOnline && (uint32_t)(now - s_lsmRetry) >= SENSOR_RETRY_MS) {
            s_lsmRetry = now;
            s_lsmOnline = LSM6DS3TR_Init(&lsm, &hspi2, LSM_CS_GPIO_Port, LSM_CS_Pin);
            s_lsmLastData = HAL_GetTick();
        }
        if (s_lsmOnline) {
            bool ok = LSM6DS3TR_DataReady(&lsm, &ready);
            if (ok && ready) {
                LSM6DS3TR_RawData_t raw;
                ok = LSM6DS3TR_ReadRaw(&lsm, &raw);
                if (ok) {
                    LSM6DS3TR_ConvertToPhysical(&lsm, &raw, sample.accel_g, sample.gyro_dps);
                    sample.temperature_c = LSM6DS3TR_TemperatureC(raw.temp_raw);
                    sample.temperature_valid = isfinite(sample.temperature_c);
                    sample.valid = IsAccelPlausible(sample.accel_g);
                    for (int i = 0; i < 3; ++i)
                        sample.valid = sample.valid && isfinite(sample.gyro_dps[i]);
                    sample.timestamp_ms = HAL_GetTick();
                    s_lsmLastData = sample.timestamp_ms;
                    xQueueOverwrite(xQueue_LSM6DS3TR, &sample);
                }
            }
            if (!ok || (uint32_t)(HAL_GetTick() - s_lsmLastData) > SENSOR_STALE_MS) {
                s_lsmOnline = false;
                s_lsmRetry = HAL_GetTick();
                sample.valid = false;
                xQueueOverwrite(xQueue_LSM6DS3TR, &sample);
            }
        }

        memset(&sample, 0, sizeof(sample));
        ready = false;
        now = HAL_GetTick();
        if (!s_mpuOnline && (uint32_t)(now - s_mpuRetry) >= SENSOR_RETRY_MS) {
            s_mpuRetry = now;
            s_mpuOnline = MPU6500_Init(&mpu, &hspi1, MPU_CS_GPIO_Port, MPU_CS_Pin);
            s_mpuLastData = HAL_GetTick();
        }
        if (s_mpuOnline) {
            bool ok = MPU6500_DataReady(&mpu, &ready);
            if (ok && ready) {
                MPU6500_RawData_t raw;
                ok = MPU6500_ReadRaw(&mpu, &raw);
                if (ok) {
                    MPU6500_ConvertToPhysical(&mpu, &raw, sample.accel_g, sample.gyro_dps);
                    sample.temperature_c = MPU6500_TemperatureC(raw.temp_raw);
                    sample.temperature_valid = isfinite(sample.temperature_c);
                    sample.valid = IsAccelPlausible(sample.accel_g);
                    for (int i = 0; i < 3; ++i)
                        sample.valid = sample.valid && isfinite(sample.gyro_dps[i]);
                    sample.timestamp_ms = HAL_GetTick();
                    s_mpuLastData = sample.timestamp_ms;
                    xQueueOverwrite(xQueue_MPU6500, &sample);
                }
            }
            if (!ok || (uint32_t)(HAL_GetTick() - s_mpuLastData) > SENSOR_STALE_MS) {
                s_mpuOnline = false;
                s_mpuRetry = HAL_GetTick();
                sample.valid = false;
                xQueueOverwrite(xQueue_MPU6500, &sample);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task_IMU_Start(void)
{
    if (Task_IMU_IsRunning())
        return;

    xQueue_BMI270 = xQueueCreate(1, sizeof(IMU_Sample_t));
    xQueue_LSM6DS3TR = xQueueCreate(1, sizeof(IMU_Sample_t));
    xQueue_MPU6500 = xQueueCreate(1, sizeof(IMU_Sample_t));

    if (!xQueue_BMI270 || !xQueue_LSM6DS3TR || !xQueue_MPU6500) Error_Handler();

    BaseType_t xReturned = xTaskCreate(
        vTask_IMU_BodyFunction,
        TASK_IMU_NAME,
        TASK_IMU_STACK_SIZE,
        NULL,
        TASK_IMU_PRIORITY,
        &xTask_IMUHandle
    );

    if (xReturned != pdPASS) {
        xTask_IMUHandle = NULL;
    }
}

void Task_IMU_Stop(void)
{
    if (xTask_IMUHandle != NULL) {
        vTaskDelete(xTask_IMUHandle);
        xTask_IMUHandle = NULL;
    }
}

bool Task_IMU_IsRunning(void)
{
    if (xTask_IMUHandle == NULL)
        return false;
    return (eTaskGetState(xTask_IMUHandle) != eDeleted);
}

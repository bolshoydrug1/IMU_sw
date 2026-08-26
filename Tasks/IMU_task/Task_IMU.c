#include "Task_IMU.h"
#include "imu_common.h"
#include "bmi270.h"
#include "lsm6ds3tr.h"
#include "mpu6500.h"
#include "main.h"

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

#define SENSOR_INIT_RETRIES 20

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
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_IMU_PERIOD_MS);

    for (;;)
    {
        IMU_Sample_t sample;

        BMI270_RawData_t bmiRaw;
        sample.valid = BMI270_ReadRaw(&bmi, &bmiRaw);
        if (sample.valid) {
            BMI270_ConvertToPhysical(&bmi, &bmiRaw, sample.accel_g, sample.gyro_dps);
            sample.valid = IsAccelPlausible(sample.accel_g);
        }
        sample.timestamp_ms = HAL_GetTick();
        xQueueOverwrite(xQueue_BMI270, &sample);

        LSM6DS3TR_RawData_t lsmRaw;
        sample.valid = LSM6DS3TR_ReadRaw(&lsm, &lsmRaw);
        if (sample.valid) {
            LSM6DS3TR_ConvertToPhysical(&lsm, &lsmRaw, sample.accel_g, sample.gyro_dps);
            sample.valid = IsAccelPlausible(sample.accel_g);
        }
        sample.timestamp_ms = HAL_GetTick();
        xQueueOverwrite(xQueue_LSM6DS3TR, &sample);

        MPU6500_RawData_t mpuRaw;
        sample.valid = MPU6500_ReadRaw(&mpu, &mpuRaw);
        if (sample.valid) {
            MPU6500_ConvertToPhysical(&mpu, &mpuRaw, sample.accel_g, sample.gyro_dps);
            sample.valid = IsAccelPlausible(sample.accel_g);
        }
        sample.timestamp_ms = HAL_GetTick();
        xQueueOverwrite(xQueue_MPU6500, &sample);

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

    /* Инициализация датчиков при старте — вызывается до старта планировщика
     * (main.c, USER CODE RTOS_THREADS), поэтому только HAL_Delay, не vTaskDelay.
     * Ограниченное число попыток, чтобы один неисправный/неподключённый датчик
     * не подвешивал загрузку прошивки навсегда.
     * DO1 включён на всё время инициализации — удобно смотреть длительность
     * осциллографом/логическим анализатором. */
    HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_SET);

    HAL_Delay(100);
    int attempts;

    attempts = 0;
    while (!BMI270_Init(&bmi, &hspi3, BMI270_CS_GPIO_Port, BMI270_CS_Pin) && attempts < SENSOR_INIT_RETRIES) {
        HAL_Delay(10);
        attempts++;
    }

    attempts = 0;
    while (!LSM6DS3TR_Init(&lsm, &hspi2, LSM_CS_GPIO_Port, LSM_CS_Pin) && attempts < SENSOR_INIT_RETRIES) {
        HAL_Delay(10);
        attempts++;
    }

    attempts = 0;
    while (!MPU6500_Init(&mpu, &hspi1, MPU_CS_GPIO_Port, MPU_CS_Pin) && attempts < SENSOR_INIT_RETRIES) {
        HAL_Delay(10);
        attempts++;
    }

    HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, GPIO_PIN_RESET);

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

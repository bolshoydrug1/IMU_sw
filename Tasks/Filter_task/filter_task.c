#include "filter_task.h"
#include "imu_common.h"
#include "Task_IMU.h"
#include "main.h"
#include <math.h>
#include <string.h>

static TaskHandle_t xTask_FilterHandle = NULL;

QueueHandle_t xQueue_FusionOutput;

#define SAMPLE_STALE_MS         10     /* старше — считаем датчик "молчащим" в этом цикле */
#define MADGWICK_BETA           0.1f   /* коэффициент коррекции по акселерометру */
#define AGREEMENT_COS_THRESHOLD 0.999f /* ~2.6 градуса между кватернионами — подобрать по факту */

#define IDX_BMI 0
#define IDX_LSM 1
#define IDX_MPU 2
#define SENSOR_COUNT 3

/* Кватернион на каждый датчик + метка времени последнего обновления для расчёта dt. */
static float s_q[SENSOR_COUNT][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f, 0.0f},
};
static uint32_t s_lastUpdateMs[SENSOR_COUNT];
/* Последние сырые accel/gyro каждого датчика — нужны для усреднения в
 * FusionOutput_t наравне с кватернионом, а не только для шага фильтра. */
static float s_lastGyro[SENSOR_COUNT][3];
static float s_lastAccel[SENSOR_COUNT][3];

/* Классический градиентно-спусковый фильтр Madgwick (IMU-вариант, без магнитометра).
 * Общественное достояние, алгоритм S. Madgwick, x-io Technologies. */
static void MadgwickUpdateIMU(float *q, float gx, float gy, float gz,
                               float ax, float ay, float az, float dt)
{
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= MADGWICK_BETA * s0;
        qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2;
        qDot4 -= MADGWICK_BETA * s3;
    }

    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * recipNorm; q[1] = q1 * recipNorm; q[2] = q2 * recipNorm; q[3] = q3 * recipNorm;
}

static void QuaternionToEuler(const float *q, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    *yaw_deg = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);
}

static bool ReadFresh(QueueHandle_t queue, uint32_t now, IMU_Sample_t *out)
{
    if (xQueuePeek(queue, out, 0) != pdTRUE) return false;
    if (!out->valid) return false;
    if ((now - out->timestamp_ms) > SAMPLE_STALE_MS) return false;
    return true;
}

static void vTask_Filter_BodyFunction(void *pvParameters)
{
    (void)pvParameters;

    uint32_t now = HAL_GetTick();
    for (int i = 0; i < SENSOR_COUNT; i++) s_lastUpdateMs[i] = now;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_FILTER_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        now = HAL_GetTick();
        bool fresh[SENSOR_COUNT] = { false, false, false };
        IMU_Sample_t sample;

        if (ReadFresh(xQueue_BMI270, now, &sample)) {
            float dt = (float)(now - s_lastUpdateMs[IDX_BMI]) / 1000.0f;
            if (dt <= 0.0f || dt > 0.2f) dt = (float)TASK_FILTER_PERIOD_MS / 1000.0f;
            s_lastUpdateMs[IDX_BMI] = now;
            MadgwickUpdateIMU(s_q[IDX_BMI],
                               sample.gyro_dps[0] * (float)M_PI / 180.0f,
                               sample.gyro_dps[1] * (float)M_PI / 180.0f,
                               sample.gyro_dps[2] * (float)M_PI / 180.0f,
                               sample.accel_g[0], sample.accel_g[1], sample.accel_g[2], dt);
            memcpy(s_lastGyro[IDX_BMI], sample.gyro_dps, sizeof(s_lastGyro[IDX_BMI]));
            memcpy(s_lastAccel[IDX_BMI], sample.accel_g, sizeof(s_lastAccel[IDX_BMI]));
            fresh[IDX_BMI] = true;
        }

        if (ReadFresh(xQueue_LSM6DS3TR, now, &sample)) {
            float dt = (float)(now - s_lastUpdateMs[IDX_LSM]) / 1000.0f;
            if (dt <= 0.0f || dt > 0.2f) dt = (float)TASK_FILTER_PERIOD_MS / 1000.0f;
            s_lastUpdateMs[IDX_LSM] = now;
            MadgwickUpdateIMU(s_q[IDX_LSM],
                               sample.gyro_dps[0] * (float)M_PI / 180.0f,
                               sample.gyro_dps[1] * (float)M_PI / 180.0f,
                               sample.gyro_dps[2] * (float)M_PI / 180.0f,
                               sample.accel_g[0], sample.accel_g[1], sample.accel_g[2], dt);
            memcpy(s_lastGyro[IDX_LSM], sample.gyro_dps, sizeof(s_lastGyro[IDX_LSM]));
            memcpy(s_lastAccel[IDX_LSM], sample.accel_g, sizeof(s_lastAccel[IDX_LSM]));
            fresh[IDX_LSM] = true;
        }

        if (ReadFresh(xQueue_MPU6500, now, &sample)) {
            float dt = (float)(now - s_lastUpdateMs[IDX_MPU]) / 1000.0f;
            if (dt <= 0.0f || dt > 0.2f) dt = (float)TASK_FILTER_PERIOD_MS / 1000.0f;
            s_lastUpdateMs[IDX_MPU] = now;
            MadgwickUpdateIMU(s_q[IDX_MPU],
                               sample.gyro_dps[0] * (float)M_PI / 180.0f,
                               sample.gyro_dps[1] * (float)M_PI / 180.0f,
                               sample.gyro_dps[2] * (float)M_PI / 180.0f,
                               sample.accel_g[0], sample.accel_g[1], sample.accel_g[2], dt);
            memcpy(s_lastGyro[IDX_MPU], sample.gyro_dps, sizeof(s_lastGyro[IDX_MPU]));
            memcpy(s_lastAccel[IDX_MPU], sample.accel_g, sizeof(s_lastAccel[IDX_MPU]));
            fresh[IDX_MPU] = true;
        }

        /* Голосование: попарное сравнение кватернионов среди "свежих" датчиков.
         * |dot| близко к 1 => совпадающая ориентация (учёт двойного покрытия кватернионов). */
        uint8_t faultMask = 0;
        int freshCount = 0;
        for (int i = 0; i < SENSOR_COUNT; i++) if (fresh[i]) freshCount++;

        if (freshCount >= 3) {
            float dot01 = fabsf(s_q[0][0]*s_q[1][0] + s_q[0][1]*s_q[1][1] + s_q[0][2]*s_q[1][2] + s_q[0][3]*s_q[1][3]);
            float dot02 = fabsf(s_q[0][0]*s_q[2][0] + s_q[0][1]*s_q[2][1] + s_q[0][2]*s_q[2][2] + s_q[0][3]*s_q[2][3]);
            float dot12 = fabsf(s_q[1][0]*s_q[2][0] + s_q[1][1]*s_q[2][1] + s_q[1][2]*s_q[2][2] + s_q[1][3]*s_q[2][3]);

            bool agree01 = dot01 > AGREEMENT_COS_THRESHOLD;
            bool agree02 = dot02 > AGREEMENT_COS_THRESHOLD;
            bool agree12 = dot12 > AGREEMENT_COS_THRESHOLD;

            /* Ровно один датчик расходится с обоими остальными, а те двое согласны
             * между собой — тогда отбраковываем именно его. Неоднозначные паттерны
             * (все согласны, либо несогласованность не сводится к одному датчику)
             * никого не бракуют — безопасный дефолт при недостатке информации. */
            if (!agree01 && !agree02 && agree12)      faultMask = (1 << IDX_BMI);
            else if (!agree01 && agree02 && !agree12) faultMask = (1 << IDX_LSM);
            else if (agree01 && !agree02 && !agree12) faultMask = (1 << IDX_MPU);
        }

        /* Итоговая ориентация и сырые accel/gyro — усреднение по датчикам,
         * не отбракованным голосованием (тем же набором, что вошёл в q). */
        float qsum[4] = { 0, 0, 0, 0 };
        float gyroSum[3] = { 0, 0, 0 };
        float accelSum[3] = { 0, 0, 0 };
        int used = 0;
        for (int i = 0; i < SENSOR_COUNT; i++) {
            if (!fresh[i]) continue;
            if (faultMask & (1 << i)) continue;
            float sign = 1.0f;
            if (used > 0) {
                float dot = qsum[0]*s_q[i][0] + qsum[1]*s_q[i][1] + qsum[2]*s_q[i][2] + qsum[3]*s_q[i][3];
                if (dot < 0) sign = -1.0f;
            }
            qsum[0] += sign * s_q[i][0];
            qsum[1] += sign * s_q[i][1];
            qsum[2] += sign * s_q[i][2];
            qsum[3] += sign * s_q[i][3];
            gyroSum[0] += s_lastGyro[i][0];
            gyroSum[1] += s_lastGyro[i][1];
            gyroSum[2] += s_lastGyro[i][2];
            accelSum[0] += s_lastAccel[i][0];
            accelSum[1] += s_lastAccel[i][1];
            accelSum[2] += s_lastAccel[i][2];
            used++;
        }

        FusionOutput_t result;
        memset(&result, 0, sizeof(result));
        result.timestamp_ms = now;
        result.fault_mask = faultMask;

        if (used > 0) {
            float norm = sqrtf(qsum[0]*qsum[0] + qsum[1]*qsum[1] + qsum[2]*qsum[2] + qsum[3]*qsum[3]);
            if (norm > 1e-6f) {
                result.q[0] = qsum[0] / norm;
                result.q[1] = qsum[1] / norm;
                result.q[2] = qsum[2] / norm;
                result.q[3] = qsum[3] / norm;
                QuaternionToEuler(result.q, &result.roll_deg, &result.pitch_deg, &result.yaw_deg);
            }
            result.gyro_dps[0] = gyroSum[0] / (float)used;
            result.gyro_dps[1] = gyroSum[1] / (float)used;
            result.gyro_dps[2] = gyroSum[2] / (float)used;
            result.accel_g[0] = accelSum[0] / (float)used;
            result.accel_g[1] = accelSum[1] / (float)used;
            result.accel_g[2] = accelSum[2] / (float)used;
        }

        xQueueOverwrite(xQueue_FusionOutput, &result);
    }
}

void Task_Filter_Start(void)
{
    if (Task_Filter_IsRunning())
        return;

    xQueue_FusionOutput = xQueueCreate(1, sizeof(FusionOutput_t));

    BaseType_t xReturned = xTaskCreate(
        vTask_Filter_BodyFunction,
        TASK_FILTER_NAME,
        TASK_FILTER_STACK_SIZE,
        NULL,
        TASK_FILTER_PRIORITY,
        &xTask_FilterHandle
    );

    if (xReturned != pdPASS) {
        xTask_FilterHandle = NULL;
    }
}

void Task_Filter_Stop(void)
{
    if (xTask_FilterHandle != NULL) {
        vTaskDelete(xTask_FilterHandle);
        xTask_FilterHandle = NULL;
    }
}

bool Task_Filter_IsRunning(void)
{
    if (xTask_FilterHandle == NULL)
        return false;
    return (eTaskGetState(xTask_FilterHandle) != eDeleted);
}

#ifndef TASK_IMU_TASK_IMU_H_
#define TASK_IMU_TASK_IMU_H_

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define TASK_IMU_STACK_SIZE (configMINIMAL_STACK_SIZE * 3)
#define TASK_IMU_PRIORITY   (tskIDLE_PRIORITY + 4)
#define TASK_IMU_NAME       "Task_IMU"
#define TASK_IMU_PERIOD_MS  10

/* Очереди длины 1 ("последнее значение"): задача пишет через xQueueOverwrite,
 * потребители (Task_Filter) читают через xQueuePeek — значение остаётся
 * доступным между обновлениями, не требует отдельного мьютекса. */
extern QueueHandle_t xQueue_BMI270;
extern QueueHandle_t xQueue_LSM6DS3TR;
extern QueueHandle_t xQueue_MPU6500;

void Task_IMU_Start(void);
void Task_IMU_Stop(void);
bool Task_IMU_IsRunning(void);

#endif /* TASK_IMU_TASK_IMU_H_ */

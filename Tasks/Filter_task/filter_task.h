#ifndef FILTER_TASK_FILTER_TASK_H_
#define FILTER_TASK_FILTER_TASK_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define TASK_FILTER_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define TASK_FILTER_PRIORITY   (tskIDLE_PRIORITY + 3)
#define TASK_FILTER_NAME       "Task_Filter"
#define TASK_FILTER_PERIOD_MS  5

typedef struct {
    float q[4];             /* w,x,y,z — усреднённая ориентация после голосования */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint8_t fault_mask;     /* бит0=BMI270, бит1=LSM6DS3TR, бит2=MPU6500 — отбракован голосованием */
    uint32_t timestamp_ms;
} FusionOutput_t;

/* Очередь длины 1 ("последнее значение"), заполняется через xQueueOverwrite,
 * Task_UART читает через xQueuePeek. */
extern QueueHandle_t xQueue_FusionOutput;

void Task_Filter_Start(void);
void Task_Filter_Stop(void);
bool Task_Filter_IsRunning(void);

#endif /* FILTER_TASK_FILTER_TASK_H_ */

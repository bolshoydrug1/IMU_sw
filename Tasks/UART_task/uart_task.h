#ifndef UART_TASK_UART_TASK_H_
#define UART_TASK_UART_TASK_H_

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

#define TASK_UART_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define TASK_UART_PRIORITY   (tskIDLE_PRIORITY + 2)
#define TASK_UART_NAME       "Task_UART"
#define TASK_UART_PERIOD_MS  50

void Task_UART_Start(void);
void Task_UART_Stop(void);
bool Task_UART_IsRunning(void);

#endif /* UART_TASK_UART_TASK_H_ */

#ifndef UART_TASK_UART_TASK_H_
#define UART_TASK_UART_TASK_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Protocol_IMU.h"

#define TASK_UART_STACK_SIZE (configMINIMAL_STACK_SIZE * 4)
#define TASK_UART_PRIORITY   (tskIDLE_PRIORITY + 2)
#define TASK_UART_NAME       "Task_UART"

/* Режим по умолчанию — 500Гц (см. Protocol_Mode_t). Пока команда 0x07
 * (MODE_CONFIG) не реализована на приём, изменить режим можно только
 * через Task_UART_SetMode() из кода. */
#define TASK_UART_DEFAULT_MODE PROTOCOL_MODE_500HZ

void Task_UART_Start(void);
void Task_UART_Stop(void);
bool Task_UART_IsRunning(void);

/* Задел под будущую команду MODE_CONFIG (0x07) — переключает частоту отправки
 * телеметрии на лету. ON_REQUEST пока не реализован (нет приёма команд). */
void Task_UART_SetMode(uint8_t mode);

#endif /* UART_TASK_UART_TASK_H_ */

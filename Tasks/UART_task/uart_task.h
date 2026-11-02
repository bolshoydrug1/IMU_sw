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

/* Режим по умолчанию, пока хост не прислал MODE_CONFIG (0x07). */
#define TASK_UART_DEFAULT_MODE PROTOCOL_MODE_500HZ

/* Poll RX independently of the telemetry rate in every mode. */
#define TASK_UART_RX_POLL_MS 2

void Task_UART_Start(void);
void Task_UART_Stop(void);
bool Task_UART_IsRunning(void);

/* Текущий режим передачи (Protocol_Mode_t). */
uint8_t Task_UART_GetMode(void);

/* Меняет режим телеметрии. Неподдерживаемое значение игнорируется. */
void Task_UART_SetMode(uint8_t mode);

/* Вызывается из USART1_IRQHandler. Приём ведётся напрямую через регистры
 * USART1, а не через HAL_UART_Receive_IT: в используемой версии HAL у
 * блокирующей передачи и приёма общий huart->Lock, из-за чего фоновый приём
 * приводил бы к HAL_BUSY в HAL_UART_Transmit. */
void Task_UART_RxIsrHandler(void);

#endif /* UART_TASK_UART_TASK_H_ */

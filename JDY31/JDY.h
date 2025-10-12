#ifndef JDY31_H
#define JDY31_H
#include "stm31f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "main.h"
#include "adc.h"
#include "OLED.h"
#include "cmsis_os.h"

#define JDY31_USART    USART2
#define JDY31_USART_CLK_ENABLE()  __HAL_RCC_USART1_CLK_ENABLE()

#define JDY31_TX_PIN    GPIO_PIN_3
#define JDY31_RX_PIN    GPIO_PIN_2
#define JDY31_GPIO_PORT    GPIOA
#define JDY31_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define JDY31_TX_TASK_PRIORITY  3
#define JDY31_RX_TASK_PRIORITY  3
#define JDY31_TX_TASK_STACK_SIZE  128
#define JDY31_RX_TASK_STACK_SIZE  128

#define JDY31_TX_QUEUE_SIZE    10
#define JDY31_RX_QUEUE_SIZE    10
#define JDY31_DATA_MAX_LENGTH    64

extern QueueHandle_t jdy31_tx_queue;
extern QueueHandle_t jdy31_rx_queue;
extern void JDY31_RxTask(void *pvParameters);
extern void JDY31_TxTask(void *pvParameters);

typedef struct
{
  uint8_t data[JDY31_DATA_MAX_LENGTH];
  uint16_t length;
}JDY31_DataFrame_t;

void JDY31_Init(UART_HandleTypeDef *huart);
BaseType_t JDY31_SendData(uint8_t *data,uint16_t length);
BaseType_t JDY31_SendATCommand(const char *command,char *response,uint16_t resp_len,uint32_t timeout);
QueueHandle_t JDY31_GetRxQueue(void);
void Process_Buletooth_Data(char *data);

#endif

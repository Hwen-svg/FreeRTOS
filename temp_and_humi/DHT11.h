#ifndef __DHT11_H
#define __DHT11_H
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

typedef enum{
  DHT_OK=0,
  DHT_TIMEOUT,
  DHT_CHECKSUM_ERROR,
  DHT_ERROR
}DHT_StatusTypeDef;

void delay_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void DHT11_Init(void);
uint8_t DHT11_Read_Data(float *temp,float *humi);
#endif

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jdy.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "OLED.h"
#include "queue.h"
#include "buzzer.h"
#include "string.h"
#include "dht11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    float temperature;  // �¶�
    float humidity;     // ʪ��
    uint16_t light; // 
} SensorData;

char send_buf[64];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern osMessageQueueId_t JDY_TX_QueueHandle;  // 新增：蓝牙发送队列
extern osMessageQueueId_t JDY_RX_QueueHandle;  // 新增：蓝牙接收队列

osMessageQueueId_t sensor_QueueHandle;
const osMessageQueueAttr_t sensor_Queue_attributes = {
  .name = "sensor_Queue"
};
/* USER CODE END Variables */
/* Definitions for sensor_Task */
osThreadId_t sensor_TaskHandle;
const osThreadAttr_t sensor_Task_attributes = {
  .name = "sensor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for oled_Task */
osThreadId_t oled_TaskHandle;
const osThreadAttr_t oled_Task_attributes = {
  .name = "oled_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for feng_Task */
osThreadId_t feng_TaskHandle;
const osThreadAttr_t feng_Task_attributes = {
  .name = "feng_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for JDY_TX_Task */
osThreadId_t JDY_TX_TaskHandle;
const osThreadAttr_t JDY_TX_Task_attributes = {
  .name = "JDY_TX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for JDY_RX_Task */
osThreadId_t JDY_RX_TaskHandle;
const osThreadAttr_t JDY_RX_Task_attributes = {
  .name = "JDY_RX_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for bt_cmd_Queue */
osMessageQueueId_t bt_cmd_QueueHandle;
const osMessageQueueAttr_t bt_cmd_Queue_attributes = {
  .name = "bt_cmd_Queue"
};
/* Definitions for JDY_TX_Queue */
osMessageQueueId_t JDY_TX_QueueHandle;
const osMessageQueueAttr_t JDY_TX_Queue_attributes = {
  .name = "JDY_TX_Queue"
};
/* Definitions for JDY_RX_Queue */
osMessageQueueId_t JDY_RX_QueueHandle;
const osMessageQueueAttr_t JDY_RX_Queue_attributes = {
  .name = "JDY_RX_Queue"
};
/* Definitions for I2C_Mutex */
osMutexId_t I2C_MutexHandle;
const osMutexAttr_t I2C_Mutex_attributes = {
  .name = "I2C_Mutex"
};
/* Definitions for UART_Mutex */
osMutexId_t UART_MutexHandle;
const osMutexAttr_t UART_Mutex_attributes = {
  .name = "UART_Mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osThreadId_t jdy31_RxTaskHandle;
const osThreadAttr_t jdy31_RxTask_attributes = {
  .name = "jdy31_RxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t jdy31_TxTaskHandle;
const osThreadAttr_t jdy31_TxTask_attributes = {
  .name = "jdy31_TxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

uint8_t bt_connected = 0;
/* USER CODE END FunctionPrototypes */

void Start_sensor_Task(void *argument);
void Start_oled_Task(void *argument);
void Start_feng_Task(void *argument);
void Start_TX_Task(void *argument);
void Start_RX_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of I2C_Mutex */
  I2C_MutexHandle = osMutexNew(&I2C_Mutex_attributes);

  /* creation of UART_Mutex */
  UART_MutexHandle = osMutexNew(&UART_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of bt_cmd_Queue */
  bt_cmd_QueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &bt_cmd_Queue_attributes);

  /* creation of JDY_TX_Queue */
  JDY_TX_QueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &JDY_TX_Queue_attributes);

  /* creation of JDY_RX_Queue */
  JDY_RX_QueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &JDY_RX_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	sensor_QueueHandle = osMessageQueueNew (10, sizeof(SensorData), &sensor_Queue_attributes);
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sensor_Task */
  sensor_TaskHandle = osThreadNew(Start_sensor_Task, NULL, &sensor_Task_attributes);

  /* creation of oled_Task */
  oled_TaskHandle = osThreadNew(Start_oled_Task, NULL, &oled_Task_attributes);

  /* creation of feng_Task */
  //feng_TaskHandle = osThreadNew(Start_feng_Task, NULL, &feng_Task_attributes);

  /* creation of JDY_TX_Task */
  JDY_TX_TaskHandle = osThreadNew(Start_TX_Task, NULL, &JDY_TX_Task_attributes);

  /* creation of JDY_RX_Task */
  JDY_RX_TaskHandle = osThreadNew(Start_RX_Task, NULL, &JDY_RX_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	jdy31_RxTaskHandle = osThreadNew(JDY31_RxTask, NULL, &jdy31_RxTask_attributes);
	
	jdy31_TxTaskHandle = osThreadNew(JDY31_TxTask, NULL, &jdy31_TxTask_attributes);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_sensor_Task */
/**
  * @brief  Function implementing the sensor_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_sensor_Task */
void Start_sensor_Task(void *argument)
{
    SensorData data = {0};
    /* Infinite loop */
    for(;;)
    {
			data.light = read_adc();
			DHT11_Read_Data(&data.temperature,&data.humidity);
			u1_printf("light:%d,t:%.2f,h:%.2f",data.light,data.temperature,data.humidity);
			osMessageQueuePut(sensor_QueueHandle,&data,0,0);
			osDelay(1000);
    }
}

/* USER CODE BEGIN Header_Start_oled_Task */
/**
* @brief Function implementing the oled_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_oled_Task */
void Start_oled_Task(void *argument)
{
  /* USER CODE BEGIN Start_oled_Task */
	OLED_Init();
  SensorData data;
	char temp_str[20];
	char humi_str[20];
	char light_str[20];
	OLED_FullyClear();
	
  /* Infinite loop */
  for(;;)
  {
		osMessageQueueGet(sensor_QueueHandle,&data,0,osWaitForever);
		//获取I2C锁
		osMutexAcquire(I2C_MutexHandle, osWaitForever);
		
		sprintf(temp_str, "temp:%f", data.temperature);
		sprintf(humi_str, "humi:%f%%", data.humidity);
		sprintf(light_str, "light:%4hu", data.light);
		OLED_ShowStr(0,1,(uint8_t*)temp_str,1);
		OLED_ShowStr(0,10,(uint8_t*)humi_str,1);
		OLED_ShowStr(0,20,(uint8_t*)light_str,1);
		//释放I2C锁
		osMutexRelease(I2C_MutexHandle);
  }
  /* USER CODE END Start_oled_Task */
}

/* USER CODE BEGIN Header_Start_feng_Task */
/**
* @brief Function implementing the feng_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_feng_Task */
void Start_feng_Task(void *argument)
{
  /* USER CODE BEGIN Start_feng_Task */
	SensorData data;
	uint8_t buzzer = 0;
  /* Infinite loop */
  for(;;)
  {
		osMessageQueueGet(sensor_QueueHandle,&data,0,osWaitForever);
		
  }
  /* USER CODE END Start_feng_Task */
}

/* USER CODE BEGIN Header_Start_TX_Task */
/**
* @brief Function implementing the JDY_TX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_TX_Task */
void Start_TX_Task(void *argument)
{
  /* USER CODE BEGIN Start_TX_Task */
	SensorData sensor_data;
  /* Infinite loop */
  for(;;)
  {
		/*if (osMessageQueueGet(sensor_QueueHandle, &sensor_data, 0, 0) == osOK)
    {
      // 格式化传感器数据为字符串
      sprintf(send_buf, "Temp:%d, Humi:%d, Light:%d", 
              (int)sensor_data.temperature, 
              (int)sensor_data.humidity, 
              sensor_data.light);
      
      // 通过蓝牙发送数据
      JDY31_SendData((uint8_t*)send_buf, strlen(send_buf));
    }*/
		if (osMessageQueueGet(sensor_QueueHandle, &sensor_data, 0, 0) == osOK) {
            
            sprintf(send_buf, "Temp:%f, Humi:%f, Light:%d",sensor_data.temperature,sensor_data.humidity,sensor_data.light);
            BaseType_t ret = JDY31_SendData((uint8_t*)send_buf, strlen(send_buf));
        }
    osDelay(2000);
  }
  /* USER CODE END Start_TX_Task */
}

/* USER CODE BEGIN Header_Start_RX_Task */
/**
* @brief Function implementing the JDY_RX_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_RX_Task */
void Start_RX_Task(void *argument)
{
  /* USER CODE BEGIN Start_RX_Task */
    JDY31_DataFrame_t rx_frame;
    char display_str[64];  // 用于OLED显示的字符串
    uint8_t rx_buffer[128];
    /* Infinite loop */
    for(;;)
    {
        // 从蓝牙接收队列获取数据
        if (xQueueReceive(JDY31_GetRxQueue(), &rx_frame, portMAX_DELAY) == pdPASS)
        {
            u1_printf("BT Received - Len: %d, Data: %s\n", rx_frame.length, rx_frame.data);
            
            // 将接收到的数据转换为字符串
            memset(display_str, 0, sizeof(display_str));
            
            // 只复制有效长度的数据，避免缓冲区溢出
            uint16_t copy_len = (rx_frame.length < sizeof(display_str)-1) ? 
                              rx_frame.length : sizeof(display_str)-1;
            memcpy(display_str, rx_frame.data, copy_len);
            display_str[copy_len] = '\0';
            
            // 检查连接状态
            if (strstr(display_str, "CONNECTED") != NULL) 
            {
                bt_connected = 1;
                u1_printf("Bluetooth Connected!\r\n");
            }
            else if (strstr(display_str, "DISCONNECT") != NULL)
            {
                bt_connected = 0;
                u1_printf("Bluetooth Disconnected!\r\n");
            }
            
            u1_printf("Display: %s\r\n", display_str);
            
            // 加锁保护I2C总线
            osMutexAcquire(I2C_MutexHandle, osWaitForever);
            
            // 清除之前显示的内容并显示新数据
            OLED_AreaClear(0, 30, 128, 32);  // 清除底部区域（第4行到第8行）
            /*
            // 显示蓝牙接收状态
            if (bt_connected) {
                OLED_ShowStr(0, 30, (uint8_t*)"BT:Connected ", 1);
            } else {
                OLED_ShowStr(0, 30, (uint8_t*)"BT:Disconnected", 1);
            }
            
            // 显示接收到的数据（自动换行处理）
            if (strlen(display_str) > 0) {
                // 第一行数据（第5行）
                char line1[17] = {0};  // OLED一行最多16字符
                strncpy(line1, display_str, 16);
                line1[16] = '\0';
                OLED_ShowStr(0, 40, (uint8_t*)line1, 1);
                
                // 第二行数据（第6行），如果有更多数据
                if (strlen(display_str) > 16) {
                    char line2[17] = {0};
                    strncpy(line2, display_str + 16, 16);
                    line2[16] = '\0';
                    OLED_ShowStr(0, 50, (uint8_t*)line2, 1);
                }
            }*/
            int len = HAL_UART_Receive(&huart2,rx_buffer,sizeof(rx_buffer),1000);
						if(len>0)
						{
							rx_buffer[len] = '\0';
							Process_Bluetooth_Data((char*)rx_frame.data);
						}
            osMutexRelease(I2C_MutexHandle);  // 释放I2C锁
        }
        osDelay(10);
    }
  /* USER CODE END Start_RX_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


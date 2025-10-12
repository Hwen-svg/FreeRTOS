#include "jdy.h"
#include <string.h>

// 全局句柄
static UART_HandleTypeDef *jdy31_huart;
QueueHandle_t jdy31_tx_queue;
QueueHandle_t jdy31_rx_queue;
static SemaphoreHandle_t jdy31_rx_semaphore;

// 接收缓冲区
static uint8_t rx_buffer[JDY31_DATA_MAX_LENGTH];
static uint16_t rx_index = 0;

// 任务声明
void JDY31_TxTask(void *pvParameters);
void JDY31_RxTask(void *pvParameters);

/**
 * @brief  初始化JDY31蓝牙模块
 * @param  huart: UART句柄
 */
void JDY31_Init(UART_HandleTypeDef *huart) {
    jdy31_huart = huart;
	
		rx_index = 0;
    memset(rx_buffer, 0, JDY31_DATA_MAX_LENGTH);
    
    // 创建信号量
    jdy31_rx_semaphore = xSemaphoreCreateBinary();
    
    // 创建发送和接收队列
    jdy31_tx_queue = xQueueCreate(JDY31_TX_QUEUE_SIZE, sizeof(JDY31_DataFrame_t));
    jdy31_rx_queue = xQueueCreate(JDY31_RX_QUEUE_SIZE, sizeof(JDY31_DataFrame_t));

    // 启动UART接收中断
    HAL_UART_Receive_IT(jdy31_huart, &rx_buffer[rx_index], 1);
}

/**
 * @brief  发送数据到蓝牙模块
 * @param  data: 要发送的数据
 * @param  length: 数据长度
 * @retval 成功返回pdPASS，失败返回errQUEUE_FULL
 */
BaseType_t JDY31_SendData(uint8_t *data, uint16_t length) {
    if (data == NULL || length == 0 || length > JDY31_DATA_MAX_LENGTH) {
        return pdFALSE;
    }
    
    JDY31_DataFrame_t frame;
    memcpy(frame.data, data, length);
    frame.length = length;
    
    return xQueueSend(jdy31_tx_queue, &frame, 0);
}

/**
 * @brief  发送AT命令到蓝牙模块并等待响应
 * @param  command: AT命令
 * @param  response: 存储响应的缓冲区
 * @param  resp_len: 响应缓冲区长度
 * @param  timeout: 超时时间(ms)
 * @retval 成功返回pdPASS，失败返回pdFALSE
 */
BaseType_t JDY31_SendATCommand(const char *command, char *response, uint16_t resp_len, uint32_t timeout) {
    if (command == NULL || response == NULL || resp_len == 0) {
        return pdFALSE;
    }
    
    // 清空响应缓冲区
    memset(response, 0, resp_len);
    
    // 发送AT命令
    JDY31_DataFrame_t frame;
    strncpy((char *)frame.data, command, JDY31_DATA_MAX_LENGTH - 1);
    frame.length = strlen((char *)frame.data);
    
    // 添加回车换行
    if (frame.length + 2 < JDY31_DATA_MAX_LENGTH) {
        frame.data[frame.length++] = '\r';
        frame.data[frame.length++] = '\n';
    }
    
    if (xQueueSend(jdy31_tx_queue, &frame, pdMS_TO_TICKS(100)) != pdPASS) {
        return pdFALSE;
    }
    
    // 等待响应
    TickType_t start_tick = xTaskGetTickCount();
    JDY31_DataFrame_t rx_frame;
    
    while ((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(timeout)) {
        if (xQueueReceive(jdy31_rx_queue, &rx_frame, pdMS_TO_TICKS(100)) == pdPASS) {
            if (rx_frame.length < resp_len) {
                memcpy(response, rx_frame.data, rx_frame.length);
                response[rx_frame.length] = '\0';
                return pdPASS;
            }
        }
    }
    
    return pdFALSE;
}

/**
 * @brief  获取接收队列句柄
 * @retval 接收队列句柄
 */
QueueHandle_t JDY31_GetRxQueue(void) {
    return jdy31_rx_queue;
}

/**
 * @brief  蓝牙发送任务
 * @param  pvParameters: 任务参数
 */
void JDY31_TxTask(void *pvParameters) {
    JDY31_DataFrame_t frame;
    
    while (1) {
        // 等待发送队列中有数据
        if (xQueueReceive(jdy31_tx_queue, &frame, portMAX_DELAY) == pdPASS) {
					
            // 发送数据
            HAL_UART_Transmit(jdy31_huart, frame.data, frame.length, HAL_MAX_DELAY);
        }
    }
}

/**
 * @brief  蓝牙接收任务
 * @param  pvParameters: 任务参数
 */
void JDY31_RxTask(void *pvParameters) {
    JDY31_DataFrame_t frame;
    
    while (1) {
        if (xSemaphoreTake(jdy31_rx_semaphore, portMAX_DELAY) == pdPASS) {
            // 调试：打印原始数据
            u1_printf("Raw RX - Len:%d, Data:", rx_index);
            for(int i = 0; i < rx_index; i++) {
                u1_printf("%02X ", rx_buffer[i]);
                if (rx_buffer[i] == 0x00) {
                    u1_printf("[NULL] ");
                }
            }
            u1_printf("\r\n");
            
            // 查找有效数据的开始和结束位置
            uint16_t data_start = 0;
            uint16_t data_end = rx_index;
            
            // 跳过前面的0x00字节
            while (data_start < rx_index && (rx_buffer[data_start] == 0x00 || 
                   rx_buffer[data_start] == '\r' || rx_buffer[data_start] == '\n')) {
                data_start++;
            }
            
            // 从后往前找有效数据的结束位置
            while (data_end > data_start && (rx_buffer[data_end-1] == 0x00 || 
                   rx_buffer[data_end-1] == '\r' || rx_buffer[data_end-1] == '\n')) {
                data_end--;
            }
            
            uint16_t valid_length = data_end - data_start;
            u1_printf("Valid data - Start:%d, End:%d, Len:%d\r\n", 
                      data_start, data_end, valid_length);
            
            if (valid_length > 0 && valid_length <= JDY31_DATA_MAX_LENGTH) {
                frame.length = valid_length;
                memcpy(frame.data, &rx_buffer[data_start], valid_length);
                frame.data[valid_length] = '\0';
                
                u1_printf("RX String: [%s]\r\n", frame.data);
                
                // 发送到接收队列
                if (xQueueSend(jdy31_rx_queue, &frame, 0) != pdPASS) {
                    u1_printf("RX Queue Full!\r\n");
                }
            } else {
                u1_printf("No valid data to process\r\n");
            }
            
            // 重置接收缓冲区
            rx_index = 0;
            memset(rx_buffer, 0, JDY31_DATA_MAX_LENGTH);
        }
    }
}

/**
 * @brief  UART接收完成回调函数
 * @param  huart: UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (huart == jdy31_huart) {
        uint8_t received_byte = rx_buffer[rx_index];
        
        // 调试：打印每个接收到的字节
        // u1_printf("[RX] idx:%d, byte:0x%02X '%c'\r\n", rx_index, received_byte, 
        //          (received_byte >= 32 && received_byte <= 126) ? received_byte : '.');
        
        // 检查帧结束条件：换行符或缓冲区满
        if (received_byte == '\n' || rx_index >= JDY31_DATA_MAX_LENGTH - 1) {
            // 有效数据检查：避免空帧或全0帧
            if (rx_index > 0) {
                xSemaphoreGiveFromISR(jdy31_rx_semaphore, &xHigherPriorityTaskWoken);
            } else {
                // 空帧，重置索引重新开始
                rx_index = 0;
            }
        } else {
            // 继续接收下一个字节
            rx_index++;
        }
        
        // 重新启动接收中断（确保不越界）
        if (rx_index < JDY31_DATA_MAX_LENGTH) {
            HAL_UART_Receive_IT(huart, &rx_buffer[rx_index], 1);
        } else {
            // 缓冲区满，强制处理
            xSemaphoreGiveFromISR(jdy31_rx_semaphore, &xHigherPriorityTaskWoken);
        }
        
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}


void Process_Bluetooth_Data(char *data)
{
    // 方法1: 字符串比较（精确匹配）
    if(strcmp(data, "hello") == 0)
    {
        // 执行hello对应的动作
				OLED_FullyClear();
				OLED_DrawSmileFace();
    }
    else if(strcmp(data, "led_on") == 0)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
		else if(strcmp(data, "led_off") == 0)
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
}


#include "dht11.h"
#include "cmsis_os.h"
#include "usart.h"
#include "oled.h"
#include "tim.h"

static uint8_t delay_initialized = 0;
void delay_init(void)
{
  if(!delay_initialized)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  //初始化
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    delay_initialized = 1;
  }
}

void delay_us(uint32_t us)  //延时函数
{
  if(us == 0) return;
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000);
  if(ticks > (SystemCoreClock / 1000))
  {
    uint32_t yield_interval = SystemCoreClock / 2000;
    uint32_t elapsed = 0;
    while(elapsed < ticks)
      {
        uint32_t current = DWT->CYCCNT;
        if((current - start) > yield_interval)
        {
          taskYIELD();
          start = DWT->CYCCNT;  //重置起始点
          elapsed = 0;
        }else{elapsed = current - start;}
      }
  }else{while((DWT->CYCCNT - start) < ticks){}}
}

void delay_ms(uint32_t)
{
  if(ms == 0)return;
  if(ms >= 2)  //大于等于2ms使用FreeRTOS延时
  {
    vTaskDelay(pdMS_TO_TICKS(ms));
  }
  else{delay_us(ms * 1000);}  //小于2ms使用微秒延时
}

void DHT11_Init(void)  //初始化DHT11引脚
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = DHT11_Pin;  //配置为推挽输出
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port,&GPIO_InitStruct);
  HAL_GPIO_WritePin(DHT11_GPIO_Port,DHT11_Pin,GPIO_PIN_SET);
}

static void DHT11_Start(void)  //发送起始信号
{
  HAL_GPIO_WritePin(DHT11_GPIO_Port,DHT11_Pin,GPIO_PIN_RESET);  //主机拉低总线至少18ms
  delay_ms(20);

  HAL_GPIO_WritePin(DHT11_GPIO_Port,DHT11_Pin,GPIO_PIN_SET);  //主机释放总线，等待20-40us
  delay_us(30);

  GPIO_InitTypeDef GPIO_InitStruct = {0};  //切换为输入模式
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

static uint8_t DHT11_Check_Response(void)  //检查DHT11响应信号
{
  uint8_t response = 0;
  if(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin) == GPIO_PIN_RESET)  //等待DHT11拉低总线80us
  {
    delay_us(80);
    if(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin) == GPIO_PIN_SET)  //等待DHT11拉高总线80us
    {
      delay_us(80);
      response = 1;
    }
  }
  return response;
}

static uint8_t DHT11_Read_Bit(void)  //读取一位数据
{
  while(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin) == GPIO_PIN_RESET);  //等待低电平结束
  delay_us(40);
  if(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin) == GPIO_PIN_SET)  //高电平持续时间决定是0还是1
  {
    while(HAL_GPIO_ReadPin(DHT11_GPIO_Port,DHT11_Pin) == GPIO_PIN_SET);
    return 1;
  }else{
    return 0;
  }
}

static uint8_t DHT11_Byte(void)  //读取一个字节数据
{
  uint8_t byte = 0;
  uint8_t i;
  for(i=0;i<8;i++)
    {
      byte <<= 1;
      byte |= DHT11_Read_Bit();
    }
  return byte;
}

uint8_t DHT11_Read_Data(float *temp,float *humi)  //读取DHT11数据
{
  uint8_t buf[5];
  uint8_t i;
  DHT11_Start();  //发送起始信号
  if(!DHT11_Check_Response())  //检查响应
  {
    return 0;
  }
  for(i=0;i<5;i++)  //读取40位数据
    {
      buf[i] = DHT11_Read_Byte();
    }
  GPIO_InitTypeDef GPIO_InitStruct = {0};  //恢复引脚为输出模式
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
  if(buf[0]+buf[1]+buf[2]+buf[3] == buf[4])  //校验和检查
  {
    *humi = buf[0] + buf[1] / 10.0f;  //湿度整数部分+小数部分
    *temp = buf[2] + buf[3] / 10.0f;  //温度整数部分+小数部分
    return 1;  //读取成功
  }else{
    return 0;  //校验失败，返回失败
  }
}

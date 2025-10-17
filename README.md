# 基于FreeRTOS的STM32环境监测系统

## 项目概述
基于STM32微控制器和FreeRTOS实时操作系统，实现多传感器数据采集、OLED显示和蓝牙远程控制。

## 功能特性
- ✅ 多传感器数据采集（DHT11温湿度、光敏传感器）
- ✅ OLED实时数据显示
- ✅ 蓝牙远程控制（LED控制、字符显示）
- ✅ FreeRTOS多任务管理

## 硬件要求
- STM32F103C8T6开发板
- DHT11温湿度传感器
- 光敏电阻模块
- OLED显示屏（SSD1306）
- JDY-31蓝牙模块

## 🏗️ 核心模块说明

### 任务模块 (Core/Src)
- **sensor_task.c** - 传感器数据采集任务
  - 周期性读取DHT11温湿度和光敏传感器数据
  - 通过消息队列将数据发送给显示任务
- **oled_task.c** - OLED显示任务  
  - 接收传感器数据并实时显示
  - 显示蓝牙接收到的字符信息
- **bluetooth_task.c** - 蓝牙通信任务
  - 处理JDY-31模块接收的数据
  - 解析控制指令并执行相应操作

### 驱动层 (Drivers/BSP)
- **OLED_SSD1306/** - OLED显示屏驱动
  - 基于I2C通信协议
  - 实现字符、数字和图形显示功能
- **DHT11/** - 温湿度传感器驱动
  - 单总线通信协议实现
  - 包含数据校验机制
- **Light_Sensor/** - 光敏传感器驱动  
  - ADC采集光照强度
  - 电压值到光照强度的转换
- **Bluetooth_JDY31/** - 蓝牙模块驱动
  - UART串口通信
  - 数据接收中断处理

### 系统配置
- **freertos_config.h** - FreeRTOS任务配置
  - 定义任务优先级、堆栈大小
  - 配置系统时钟和调度策略

## 相关视频图片
硬件连接图：https://github.com/Hwen-svg/OTHER/blob/main/SCH_Schematic1_1-P1_2025-10-12.png

演示视频：https://github.com/Hwen-svg/OTHER/blob/main/10%E6%9C%8817%E6%97%A5(2).mp4
注：蓝牙相关功能因设备有限暂时无法进行拍摄




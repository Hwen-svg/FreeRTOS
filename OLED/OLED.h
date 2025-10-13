#ifndef OLED_H__
#define OLED_H__

#include "stm32f1xx_hal.h"

extern const uint8_t BMP_Picture[64/8][64];

#define LEFT 0x27
#define RIGHT 0x26
#define UP 0x29
#define DOWM 0x2A
#define ON 0xA7
#define OFF 0xA6

typedef enum{
  SET_PIXEL = 0x01,
  RESET_PIXEL = 0x00,
}PixelStatus;

void HAL_I2C_WriteByte(uint8_t addr,uint8_t data);
void WriteCmd(uint8_t IIC_Command);
void WriteDat(uint8_t IIC_Data);
void OLED_Init(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_RefreshRAM(void);
void OLED_ClearRAM(void);
void OLED_FullyFill(uint8_t fill_Data);
void OLED_FullyClear(void);
void OLED_SetPixel(int16_t x,int16_t y,uint8_t set_pixel);
PixelStatus OLED_GetPixel(int16_t x,int16_t y);

void OLED_ShowStr(int16_t x, int16_t y, uint8_t ch[], uint8_t TextSize);
//显示中文字符串
void OLED_ShowCN(int16_t x, int16_t y, uint8_t* ch);
//显示中英文混合文字
void OLED_ShowMixedCH(int16_t x, int16_t y, uint8_t* ch);
//显示图片
void OLED_DrawBMP(int16_t x0,int16_t y0,int16_t L,int16_t H,const uint8_t BMP[]);

//区域填充
void OLED_AreaFill(int16_t x0,int16_t y0,int16_t L,int16_t H, uint8_t fill_data);
//区域清除
void OLED_AreaClear(int16_t x0,int16_t y0,int16_t L,int16_t H);
//全屏切换显示
void OLED_FullyToggle(void);
//区域切换显示
void OLED_AreaToggle(int16_t x0,int16_t y0,int16_t L,int16_t H);
//全屏垂直滚动播放
void OLED_VerticalShift(void);
//全屏水平滚动播放
void OLED_HorizontalShift(uint8_t direction);
//全屏同时垂直和水平滚动播放
void OLED_VerticalAndHorizontalShift(uint8_t direction);
//屏幕内容取反显示
void OLED_DisplayMode(uint8_t mode);
//屏幕亮度调节
void OLED_IntensityControl(uint8_t intensity);
//笑脸函数
void OLED_DrawSmileFace(void);

#endif

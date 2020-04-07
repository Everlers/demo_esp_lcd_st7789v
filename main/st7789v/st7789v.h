#ifndef _ST7789V_H_
#define _ST7789V_H_
#include "stdio.h"
#include "driver/gpio.h"

#define ST_MOSI_IO              GPIO_NUM_19
#define ST_SCLK_IO              GPIO_NUM_18
#define ST_CS_IO                GPIO_NUM_5
#define ST_DC_IO                GPIO_NUM_16
#define ST_RST_IO               GPIO_NUM_23
#define ST_BL_IO                GPIO_NUM_4

#define LCD_WIDTH_OFFSET        40
#define LCD_HIGH_OFFSET         53

#define LCD_WIDTH               239
#define LCD_HIGH                136

/*
 LCD需要一堆命令/参数来初始化。它们储存在这个结构中。
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

void lcd_init(void);
void lcd_blk_on(void);
void lcd_blk_off(void);
void lcd_clean(uint16_t color);
void lcd_set_frame(uint16_t xstart,uint16_t ystart,uint16_t xend,uint16_t yend);

void lcd_show_string(uint16_t x,uint16_t y,char *str,uint16_t pointColor,uint16_t backColor);
#endif
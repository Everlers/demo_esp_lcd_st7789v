#ifndef _ST7789V_H_
#define _ST7789V_H_
#include "stdio.h"
#include "driver/gpio.h"

#define ST_MOSI_IO              GPIO_NUM_19
#define ST_SCLK_IO              GPIO_NUM_18
#define ST_SC_IO                GPIO_NUM_5
#define ST_DC_IO                GPIO_NUM_16
#define ST_RST_IO               GPIO_NUM_23
#define ST_BL_IO                GPIO_NUM_4

/*
 LCD需要一堆命令/参数来初始化。它们储存在这个结构中。
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

void st7789v_init(void);
void st7789v_blk_on(void);
void st7789v_blk_off(void);
void st7789v_write_cmd(uint8_t cmd);
void st7789v_write_data(uint8_t *data,uint8_t len);
void st7789v_clean_display(void);
void st7789v_set_frame(uint16_t xstart,uint16_t ystart,uint16_t xend,uint16_t yend);
#endif
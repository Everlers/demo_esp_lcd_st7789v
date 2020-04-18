#include "st7789v.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

static spi_device_handle_t stspi;
DMA_ATTR static const lcd_init_cmd_t st_init_cmds[];
static const uint8_t font6_8[][6];

static void st7789v_write_cmd(uint8_t cmd);
static void st7789v_write_data(uint8_t *data,uint32_t len);
static void spi_pre_transfer_callback (spi_transaction_t *trans);
static void lcd_show_6x8Char(uint16_t x,uint16_t y,char chr,uint16_t pointColor,uint16_t backColor);

void lcd_init(void)
{
    esp_err_t ret;
    int cmd = 0;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    printf("ST7789V init...\r\n");
    memset(&buscfg,0,sizeof(buscfg));
    memset(&devcfg,0,sizeof(devcfg));

    buscfg.miso_io_num = -1;                    //不使用数据输入IO
    buscfg.quadhd_io_num = -1;          
    buscfg.quadwp_io_num = -1;          
    buscfg.mosi_io_num = ST_MOSI_IO;            //配置SPI输出IO
    buscfg.sclk_io_num = ST_SCLK_IO;            //配置SPI时钟IO
    buscfg.max_transfer_sz = 320*240*2;         //最大传输字节

    devcfg.mode = 0;                            //SPI模式0
    devcfg.spics_io_num = ST_CS_IO;             //CS引脚配置
    devcfg.queue_size = 7;                      //传输队列大小
    devcfg.flags = SPI_DEVICE_NO_DUMMY;         //只发送数据不接收数据 (这样SPI输出能达到80MHz)
    devcfg.pre_cb = spi_pre_transfer_callback;  //SPI传输前的回调 用于操作DC引脚
    devcfg.clock_speed_hz = SPI_MASTER_FREQ_80M;//配置SIP通讯时序 80MHz
    devcfg.cs_ena_pretrans = 1;                 //传输前CS的拉低延迟

    ret = spi_bus_initialize(SPI3_HOST,&buscfg,1);       //初始化SPI总线 不使用DMA
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI3_HOST,&devcfg,&stspi);  //将SPI添加到设备总线
    ESP_ERROR_CHECK(ret);

    gpio_set_direction(ST_DC_IO,GPIO_MODE_OUTPUT);
    gpio_set_direction(ST_RST_IO,GPIO_MODE_OUTPUT);
    gpio_set_direction(ST_BL_IO,GPIO_MODE_OUTPUT);
    
    gpio_set_level(ST_BL_IO,0);//先关闭背光板进行初始化
    
    gpio_set_level(ST_RST_IO,0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(ST_RST_IO,1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //Send all the commands
    while (st_init_cmds[cmd].databytes!=0xff) {
        st7789v_write_cmd(st_init_cmds[cmd].cmd);
        st7789v_write_data((uint8_t *)st_init_cmds[cmd].data, st_init_cmds[cmd].databytes&0x1F);
        if (st_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
    printf("ST7789V clean display\r\n");
    lcd_clean(COLOR_BLACK);
    lcd_blk_on();
    printf("ST7789V init done.\r\n");
}

void lcd_show_string(uint16_t x,uint16_t y,char *str,uint16_t pointColor,uint16_t backColor)
{
    while(*str != '\0')
    {
        if(x > LCD_WIDTH-6)
        {
            x = 0;
            y++;
        }
        if(y > LCD_HIGH-8)
            x = y = 0;
        lcd_show_6x8Char(x,y,*str,pointColor,backColor);
        x+=6;
        str++;
    }
}

static void lcd_show_6x8Char(uint16_t x,uint16_t y,char chr,uint16_t pointColor,uint16_t backColor)
{
    uint8_t index;
    uint16_t *p;
    index = chr-32;
    p = heap_caps_malloc(6*8*2,MALLOC_CAP_DMA);
    for(int yi=0;yi<8;yi++)
    {
        for(int xi=0;xi<6;xi++)
        {
            if(font6_8[index][xi] & (1<<yi))
                p[(yi * 6) + xi] = pointColor;
            else
                p[(yi * 6) + xi] = backColor;
        }
    }
    lcd_set_frame(x,y,x+6-1,y+8-1);
    st7789v_write_data((uint8_t *)p,6*8*2);
    heap_caps_free(p);
}

void lcd_clean(uint16_t color)
{
    uint8_t *p = heap_caps_malloc(LCD_WIDTH * LCD_HIGH * 2,MALLOC_CAP_DMA);
    memset(p,color,LCD_WIDTH * LCD_HIGH * 2);
    lcd_set_frame(0,0,LCD_WIDTH,LCD_HIGH);
    st7789v_write_data(p,LCD_WIDTH * LCD_HIGH * 2);
    heap_caps_free(p);
}

//设置xy
void lcd_set_frame(uint16_t xstart,uint16_t ystart,uint16_t xend,uint16_t yend)
{
    spi_transaction_t trans[5];
    xstart += LCD_WIDTH_OFFSET;
    xend += LCD_WIDTH_OFFSET;
    ystart += LCD_HIGH_OFFSET;
    yend += LCD_HIGH_OFFSET;
    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (int x=0; x<5;x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=xstart>>8;      //Start Col High
    trans[1].tx_data[1]=xstart&0xff;    //Start Col Low
    trans[1].tx_data[2]=xend>>8;        //End Col High
    trans[1].tx_data[3]=xend&0xff;      //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ystart>>8;      //Start page high
    trans[3].tx_data[1]=ystart&0xff;    //start page low
    trans[3].tx_data[2]=yend>>8;        //end page high
    trans[3].tx_data[3]=yend&0xff;      //end page low
    trans[4].tx_data[0]=0x2C;           //memory write
    for(int i=0;i<5;i++){
        spi_device_polling_transmit(stspi,&trans[i]);
    }
}

void lcd_blk_on(void)
{
    gpio_set_level(ST_BL_IO,1);
}

void lcd_blk_off(void)
{
    gpio_set_level(ST_BL_IO,0);
}

//ST7789V 驱动部分

//SPI传输前对回调
static void spi_pre_transfer_callback (spi_transaction_t *trans)
{
    int dc = (int)trans->user;
    gpio_set_level(ST_DC_IO,dc);
}

static void st7789v_write_cmd(uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.flags = 0;
    t.length = 8;
    t.tx_buffer = &cmd;
    t.user = (void*)0;//cmd
    spi_device_polling_transmit(stspi,&t);
}

static void st7789v_write_data(uint8_t *data,uint32_t len)
{
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t,0,sizeof(t));
    t.flags = 0;
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void*)1;//data
    spi_device_polling_transmit(stspi,&t);
}

//将数据放入DRAM中，默认情况下，常量数据放入DROM中，这是DMA无法访问的。
//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.
DMA_ATTR static const lcd_init_cmd_t st_init_cmds[]={
    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
    {0x36, {(1<<5)|(1<<6)|(1<<3)}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Porch Setting */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {0xB7, {0x45}, 1},
    /* VCOM Setting, VCOM=1.175V */
    {0xBB, {0x2B}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    {0xC0, {0x18}, 1},
    /* VDV and VRH Command Enable, enable=1 */
    {0xC2, {0x01, 0xff}, 2},
    /* VRH Set, Vap=4.4+... */
    {0xC3, {0x11}, 1},
    /* VDV Set, VDV=0 */
    {0xC4, {0x20}, 1},
    /* Frame Rate Control, 60Hz, inversion=0 */
    {0xC6, {0x0f}, 1},
    /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
    {0xD0, {0xA4, 0xA1}, 1},
    /* Positive Voltage Gamma Control */
    {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},
    /* Negative Voltage Gamma Control */
    {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},
    /* Sleep Out */
    {0x11, {0}, 0x80},
    /* Display On */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}
};

//ASCII 6*8
static const uint8_t font6_8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   //sp0
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !1
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "2
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #3
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $4
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %5
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &6
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '7
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (8
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )9
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *10
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +11
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,12
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -13
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .14
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /15
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 016
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 117
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 218
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 319
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 420
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 521
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 622
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 723
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 824
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 925
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :26
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;27
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <28
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =29
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >30
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?31
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @32
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A33
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B34
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C35
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D36
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E37
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F38
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G39
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H40
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I41
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J42
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K43
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L44
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M45
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N46
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O47
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P48
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q49
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R50
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S51
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T52
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U53
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V54
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W55
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X56
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y57
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z58
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [59
    { 0x00, 0x02, 0x04, 0x08, 0x10, 0x20 },   // \60
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]61
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^62
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _63
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '64
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a65
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b66
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c67
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d68
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e69
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f70
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g71
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h72
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i73
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j74
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k75
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l76
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m77
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n78
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o79
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p80
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q81
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r82
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s83
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t84
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u85
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v86
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w87
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x88
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y89
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z90
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines91
};
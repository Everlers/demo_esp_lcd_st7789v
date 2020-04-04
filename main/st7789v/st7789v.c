#include "st7789v.h"
#include "st7789v.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

static spi_device_handle_t stspi;
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[];

static void spi_pre_transfer_callback (spi_transaction_t *trans);

void st7789v_init(void)
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
    devcfg.spics_io_num = ST_SC_IO;             //CS引脚配置
    devcfg.queue_size = 7;                      //传输队列大小
    devcfg.flags = SPI_DEVICE_NO_DUMMY;         //只发送数据不接收数据 (这样SPI输出能达到80MHz)
    devcfg.pre_cb = &spi_pre_transfer_callback; //SPI传输前的回调 用于操作DC引脚
    devcfg.clock_speed_hz = SPI_MASTER_FREQ_80M;//配置SIP通讯时许 80MHz

    ret = spi_bus_initialize(SPI3_HOST,&buscfg,1);       //初始化SPI总线 不使用DMA
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI3_HOST,&devcfg,&stspi);  //将SPI添加到设备总线
    ESP_ERROR_CHECK(ret);

    gpio_pad_select_gpio(ST_DC_IO);
    gpio_set_direction(ST_DC_IO,GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(ST_RST_IO);
    gpio_set_direction(ST_RST_IO,GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(ST_BL_IO);
    gpio_set_direction(ST_BL_IO,GPIO_MODE_OUTPUT);
    
    gpio_set_level(ST_BL_IO,0);//先关闭背光板进行初始化
    
    gpio_set_level(ST_RST_IO,0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(ST_RST_IO,1);
    //vTaskDelay(100 / portTICK_RATE_MS);

    //Send all the commands
    while (st_init_cmds[cmd].databytes!=0xff) {
        st7789v_write_cmd(st_init_cmds[cmd].cmd);
        st7789v_write_data((uint8_t *)st_init_cmds[cmd].data, st_init_cmds[cmd].databytes&0x1F);
        if (st_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
    printf("ST7789V init done.\r\n");
}

void st7789v_blk_on(void)
{
    gpio_set_level(ST_BL_IO,1);
}

void st7789v_blk_off(void)
{
    gpio_set_level(ST_BL_IO,0);
}

void st7789v_clean_display(void)
{
    uint16_t xstart = 0,xend = 320,ystart = 0,yend = 240;
    spi_transaction_t trans[6];
    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (int x=0; x<6;x++) {
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
    trans[2].tx_data[0]=0x2B;           //Page address setd
    trans[3].tx_data[0]=ystart>>8;      //Start page high
    trans[3].tx_data[1]=ystart&0xff;    //start page low
    trans[3].tx_data[2]=yend>>8;        //end page high
    trans[3].tx_data[3]=yend&0xff;      //end page low
    trans[4].tx_data[0]=0x2C;           //memory write
    trans[5].flags = 0;
    trans[5].user = (void*)1;
    trans[5].length = 320*240*2*8;
    trans[5].tx_buffer = pvPortMalloc(320*240*2);
    memset(trans[5].tx_buffer,0xff,320*240*2);
    for(int i=0;i<6;i++){
        spi_device_queue_trans(stspi,&trans[i],portMAX_DELAY);
    }
    vPortFree(trans[5].tx_buffer);
}

void st7789v_set_frame(uint16_t xstart,uint16_t ystart,uint16_t xend,uint16_t yend)
{
    spi_transaction_t trans[5];
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

//SPI传输前对回调
static void spi_pre_transfer_callback (spi_transaction_t *trans)
{
    int dc = (int)trans->user;
    gpio_set_level(ST_DC_IO,dc);
}

void st7789v_write_cmd(uint8_t cmd)
{
    spi_transaction_t t;
    memset(&t,0,sizeof(t));
    t.flags = 0;
    t.length = 8;
    t.tx_buffer = &cmd;
    t.user = (void*)0;//cmd
    spi_device_polling_transmit(stspi,&t);
}

void st7789v_write_data(uint8_t *data,uint8_t len)
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
DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[]={
    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
    {0x36, {(1<<5)|(1<<6)}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Porch Setting */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {0xB7, {0x45}, 1},
    /* VCOM Setting, VCOM=1.175V */
    {0xBB, {0x2B}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    {0xC0, {0x2C}, 1},
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
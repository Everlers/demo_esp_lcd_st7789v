#include "st7789v.h"
#include "st7789v.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

void st7789vInit(void)
{
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    
    buscfg.miso_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.mosi_io_num = GPIO_NUM_19;
    buscfg.sclk_io_num = GPIO_NUM_18;
    buscfg.max_transfer_sz = 
}
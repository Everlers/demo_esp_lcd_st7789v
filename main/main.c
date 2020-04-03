/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "st7789v.h"

uint16_t color = 0xFFFF;

void app_main(void)
{
    st7789v_init();
    st7789v_blk_on();
    st7789v_set_frame(0,0,320,240);
    for(int i=0;i<320*240;i++)
        st7789v_write_data((uint8_t *)&color,2);
    while (1)
    {
        vTaskDelay(100);
    }
}
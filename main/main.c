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

void app_main(void)
{
    st7789v_init();
    st7789v_blk_on();
    //st7789v_set_frame(0,0,240,320);
    st7789v_clean_display();
    printf("ST7789V clean done.\r\n");
    while (1)
    {
        vTaskDelay(100);
    }
}
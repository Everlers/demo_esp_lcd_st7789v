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
#include "./st7789v/st7789v.h"

void app_main(void)
{
    lcd_init();
    lcd_show_string(0,0,"ESP32",WHITE,BLACK);
    lcd_show_string(0,8,"Lcd test",WHITE,BLACK);
    lcd_show_string(0,16,"Lcd ic: ST7789V",WHITE,BLACK);
    while (1)
    {
        vTaskDelay(100);
    }
}
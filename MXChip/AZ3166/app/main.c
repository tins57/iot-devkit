/* 
 * Copyright (c) Microsoft
 * Copyright (c) 2024 Eclipse Foundation
 * 
 *  This program and the accompanying materials are made available 
 *  under the terms of the MIT license which is available at
 *  https://opensource.org/license/mit.
 * 
 *  SPDX-License-Identifier: MIT
 * 
 *  Contributors: 
 *     Microsoft         - Initial version
 *     Frédéric Desbiens - 2024 version.
 */

#include <stdio.h>

#include "tx_api.h"

#include "board_init.h"
#include "cmsis_utils.h"
#include "screen.h"
#include "sntp_client.h"
#include "wwd_networking.h"

#include "cloud_config.h"

#include "ssd1306.h"
#include "sensor.h"

void screen_print_inv(char* str, LINE_NUM line)
{
    // ssd1306_Fill(White);
    ssd1306_SetCursor(1, line);
    ssd1306_WriteString(str, Font_11x18, Black);
    // ssd1306_UpdateScreen();
}

void screen_print_inv_small(char* str, LINE_NUM line)
{
    // ssd1306_Fill(White);
    if (str[0] != ' ') {
        ssd1306_SetCursor(90, line);
        ssd1306_WriteString(str, Font_7x10, Black);
    }
    else {
        ssd1306_SetCursor(97, line);
        ssd1306_WriteString(str + 1, Font_7x10, Black);
    }
    // ssd1306_UpdateScreen();
}

#define ECLIPSETX_THREAD_STACK_SIZE 4096
#define ECLIPSETX_THREAD_PRIORITY   4

TX_THREAD eclipsetx_thread;
TX_THREAD eclipsetx_thread2;
ULONG eclipsetx_thread_stack[ECLIPSETX_THREAD_STACK_SIZE / sizeof(ULONG)];
ULONG eclipsetx_thread_stack2[ECLIPSETX_THREAD_STACK_SIZE / sizeof(ULONG)];


static void eclipsetx_thread_entry(ULONG parameter)
{
    UINT status;

    printf("Starting Eclipse ThreadX thread\r\n\r\n");

    // Initialize the network
    if ((status = wwd_network_init(WIFI_SSID, WIFI_PASSWORD, WIFI_MODE)))
    {
        printf("ERROR: Failed to initialize the network (0x%08x)\r\n", status);
    }

    ssd1306_Fill(White);
    int i = 0;
    for (i = L0 + 4; i<=L3 + 9; i+=9) {
        screen_print_inv("Success!", i);
        if (i) {
            ssd1306_UpdateScreen();
        }
    }
    
    float temp = -250.;
    // float humid = -250.;
    char s[128];
    while(1) {
        tx_thread_sleep(200);
        hts221_data_t data = hts221_data_read(); 
        lps22hb_t data_p = lps22hb_data_read();
        if (data.temperature_degC != temp) {
            UINT time = (UINT)(tx_time_get() / 100);
            temp = data.temperature_degC;
            float temp_1 = data_p.temperature_degC;


            printf("%02i:%02i:%02i : t_degC*100= %i %i; hmdt*10= %i; hPa= %i \r\n",
                (180 + 20 + time / 60) / 60,
                (180 + 20 + time / 60) % 60, time % 60,
                (int)(temp * 100), (int)(temp_1 * 100), (int)(data.humidity_perc * 10), (int) data_p.pressure_hPa);

            sprintf(s, "%02i:%02i:%02i", (time / 60) / 60,
                (time / 60) % 60, time % 60);
            screen_print_inv(s, L0);
            screen_print_inv_small(" Time", L0 + 1);

            sprintf(s, "t=%i.%02i",
                (int)(temp), ((int) (temp * 100)) % 100);
                //(int)(temp_1), ((int) (temp_1 * 10)) % 10);
            screen_print_inv(s, L1);

            sprintf(s, "/%i.%01i",
                (int)(temp_1), ((int) (temp_1 * 10)) % 10);
            screen_print_inv_small("o", L1 - 4);
            screen_print_inv_small(" C", L1);
            screen_print_inv_small(s, L1 + 9);

            sprintf(s, "%i.%01i%% %i",
                (int)(data.humidity_perc),
                ((int)(data.humidity_perc * 10)) % 10,
                (int) data_p.pressure_hPa);
            screen_print_inv(s, L2);
            screen_print_inv_small("hPa", L2 + 16);

            ssd1306_UpdateScreen();
        }
    }

}

void tx_application_define(void* first_unused_memory)
{
    systick_interval_set(TX_TIMER_TICKS_PER_SECOND);

    // Create ThreadX thread
    UINT status = tx_thread_create(&eclipsetx_thread,
        "Eclipse ThreadX Thread",
        eclipsetx_thread_entry,
        0,
        eclipsetx_thread_stack,
        ECLIPSETX_THREAD_STACK_SIZE,
        ECLIPSETX_THREAD_PRIORITY,
        ECLIPSETX_THREAD_PRIORITY,
        TX_NO_TIME_SLICE,
        TX_AUTO_START);

    if (status != TX_SUCCESS)
    {
        printf("ERROR: Eclipse ThreadX thread creation failed\r\n");
    }
}

int main(void)
{
    // Initialize the board
    board_init();

    // Enter the ThreadX kernel
    tx_kernel_enter();

    return 0;
}

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
#include <math.h>

#include "tx_api.h"

#include "board_init.h"
#include "cmsis_utils.h"
#include "screen.h"
#include "sntp_client.h"
#include "wwd_networking.h"

#include "cloud_config.h"

#include "ssd1306.h"
#include "sensor.h"

void screen_print_col(char* str, LINE_NUM line, SSD1306_COLOR color)
{
    // ssd1306_Fill(White);
    ssd1306_SetCursor(1, line);
    ssd1306_WriteString(str, Font_11x18, color);
    // ssd1306_UpdateScreen();
}

void screen_print_col_small(char* str, LINE_NUM line, SSD1306_COLOR color)
{
    // ssd1306_Fill(White);
    if (str[0] != ' ') {
        ssd1306_SetCursor(79, line + 2);
        ssd1306_WriteString(str, Font_7x10, color);
    }
    else {
        ssd1306_SetCursor(85, line + 2);
        ssd1306_WriteString(str + 1, Font_7x10, color);
    }
    // ssd1306_UpdateScreen();
}

#define ECLIPSETX_THREAD_STACK_SIZE 4096
#define ECLIPSETX_THREAD_PRIORITY   4

TX_THREAD eclipsetx_thread;
TX_THREAD eclipsetx_thread2;
ULONG eclipsetx_thread_stack[ECLIPSETX_THREAD_STACK_SIZE / sizeof(ULONG)];
ULONG eclipsetx_thread_stack2[ECLIPSETX_THREAD_STACK_SIZE / sizeof(ULONG)];

int __errno(void) {
    return 0; // Always return 0 (no error)
}

static double accel_integral = 0.;
static double angul_integral = 0.;
static double ang0_int = 0.;
static double ang1_int = 0.;
static double ang2_int = 0.;
static int shaken = 0;
static int special_screen = 0;

float max_temp = -250.;
float min_temp = 250.;
float max_humid = 0.;
float min_humid = 100.;
float max_press = 0.;
float min_press = 99999.;

static char* strs[] = {
    "local", "time ",
    "MAX  ", "mode ",
    "min  ", "mode ",
    "tilt ", "me:-)",
    "up  4", " MAX ",
    "dwn 4", " min ",
    "sdw 4", "deflt",
    "Good!", "day2u"
};

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
        screen_print_col("Success!", i, Black);
        if (i) {
            ssd1306_UpdateScreen();
        }
    }

    ssd1306_Fill(Black);
    
    float temp = -250.;
    float humid = -1.;

    char s[64];
    UINT prev_time = (UINT)(tx_time_get() / 100);
    while(1) {
        tx_thread_sleep(5);
        UINT read_time = (UINT)(tx_time_get() / 100);

        lis2mdl_data_t mag = lis2mdl_data_read();
        double m_total = (double)sqrt(mag.magnetic_mG[0] * mag.magnetic_mG[0] +
                         mag.magnetic_mG[1] * mag.magnetic_mG[1] +
                         mag.magnetic_mG[2] * mag.magnetic_mG[2]);
        double m_xy = (double)sqrt(mag.magnetic_mG[0] * mag.magnetic_mG[0] +
                         mag.magnetic_mG[1] * mag.magnetic_mG[1]);

        char c = 'N';
        char d = ' ';
        int asin_to_pole = (int) (200./3.1415927 * asin((double)mag.magnetic_mG[0] / m_xy));
        int asin_to_mdir = (int) (200./3.1415927 * asin(-(double)mag.magnetic_mG[1] / m_total));

        if ((double)mag.magnetic_mG[1] >= 0.) {
            c = 'S';
            asin_to_pole *= -1;
        }

        #define asin_threshold 50.
        if (asin_to_pole >= asin_threshold) {
            if (c == 'N') {d = 'W'; c = 'n';}
            if (c == 'S') {d = 'E'; c = 's';}
        } else if (asin_to_pole <= -asin_threshold){
            if (c == 'N') {d = 'E'; c = 'n';}
            if (c == 'S') {d = 'W'; c = 's';}
        } else if (asin_to_pole >= asin_threshold * 0.5) {
            if (c == 'N') {d = 'W'; ;}
            if (c == 'S') {d = 'E'; ;}
        } else if (asin_to_pole >= 0.) {
            if (c == 'N') {d = 'w'; ;}
            if (c == 'S') {d = 'e'; ;}
        } else if (asin_to_pole < -asin_threshold * 0.5){
            if (c == 'N') {d = 'E'; ;}
            if (c == 'S') {d = 'W'; ;}
        } else if (asin_to_pole < 0.){
            if (c == 'N') {d = 'e'; ;}
            if (c == 'S') {d = 'w'; ;}
        }

        sprintf(s, "%c%c%+3i|%+3i|%2i.", c, d,
            asin_to_pole,
            asin_to_mdir,
            (int) (log(m_total) * 10.));
        ssd1306_SetCursor(0, L3);
        ssd1306_WriteString(s, Font_7x10, White);


        lsm6dsl_data_t accel_gyro = lsm6dsl_data_read();
        double accel_total = sqrt((double)(accel_gyro.acceleration_mg[0] * accel_gyro.acceleration_mg[0] +
                      accel_gyro.acceleration_mg[1] * accel_gyro.acceleration_mg[1] +
                      accel_gyro.acceleration_mg[2] * accel_gyro.acceleration_mg[2]));
        double angul_total = sqrt((double)(accel_gyro.angular_rate_mdps[0] * accel_gyro.angular_rate_mdps[0] +
                accel_gyro.angular_rate_mdps[1] * accel_gyro.angular_rate_mdps[1] +
                accel_gyro.angular_rate_mdps[2] * accel_gyro.angular_rate_mdps[2]));
        // printf("accel_total=%i\r\n", (int) (accel_total / 80.));
        accel_integral = accel_integral * 0.9 + accel_total * 0.1;

        if (~shaken)
            angul_integral = angul_integral * 0.7 + angul_total * 0.3;

        // Shake it up
        {
            ssd1306_SetCursor(95, L1 + 1);
            int shake_val = (int) (log(angul_integral + 1.) * 5);

            // If change of recently tilted status needed
            if ((shake_val >= 58.0) && ~shaken) {
                // Just shaken
                shaken |= 1;
                ang0_int = ang0_int * 0.95 + 0.05 * (double)(accel_gyro.angular_rate_mdps[0]);
                ang1_int = ang1_int * 0.95 + 0.05 * (double)(accel_gyro.angular_rate_mdps[1]);
                ang2_int = ang2_int * 0.95 + 0.05 * (double)(accel_gyro.angular_rate_mdps[2]);
            } else if (shake_val <= 57.0) {
                shaken &= 0;
            }


            if (shaken) {
                char sh_state = '!';
                if (abs(ang2_int) >= abs(ang0_int) && (abs(ang2_int) >= abs(ang1_int))) {
                    sh_state = 'D';
                    special_screen = 0;
                }
                if (abs(ang1_int) >= abs(ang0_int) && (abs(ang1_int) >= abs(ang2_int)))
                    sh_state = 'U';
                if (abs(ang0_int) >= abs(ang1_int) && (abs(ang0_int) >= abs(ang2_int))) {
                    if (ang0_int >= 0) {
                        sh_state = 'N';
                        special_screen = 1;
                    }
                    else {
                        sh_state = 'S';
                        special_screen = 2;
                    }
                }
                
                if (sh_state == 'U')
                {
                    if (ang1_int >= 0.)
                        sh_state = 'L';
                    else
                        sh_state = 'R';
                }
                //else
                // sprintf(s, "%3iL", shake_val);

                sprintf(s, "%3i%c", shake_val, sh_state);
                // Shaken cool down
                angul_integral = angul_integral * 0.95 + angul_total * 0.05;
                ang0_int = ang0_int * 0.9 + 0.0 * (double)(accel_gyro.angular_rate_mdps[0]);
                ang1_int = ang1_int * 0.9 + 0.0 * (double)(accel_gyro.angular_rate_mdps[1]);
                ang2_int = ang2_int * 0.9 + 0.0 * (double)(accel_gyro.angular_rate_mdps[2]);
            }
            else {
                // only if not shaken recently -- standard behaviour
                ang0_int = ang0_int * 0.7 + 0.3 * (double)(accel_gyro.angular_rate_mdps[0]);
                ang1_int = ang1_int * 0.7 + 0.3 * (double)(accel_gyro.angular_rate_mdps[1]);
                ang2_int = ang2_int * 0.7 + 0.3 * (double)(accel_gyro.angular_rate_mdps[2]);
                sprintf(s, "%3i.", shake_val);
            }
            ssd1306_WriteString(s, Font_7x10, White);
        }

        
        if (prev_time == read_time) {
            // update screen for fast processes (magnetic, gyro)
            ssd1306_UpdateScreen();
            // prev_time = read_time;
            continue;
        }
        prev_time = read_time;

        hts221_data_t data = hts221_data_read(); 
        lps22hb_t data_p = lps22hb_data_read();

        UINT time = read_time + (btn_a_state() - btn_b_state()) * 60;

        { // print Time on OLED screen
            sprintf(s, "%02i:%02i:%02i", (time / 60) / 60,
                (time / 60) % 60, time % 60);
            
            if ((special_screen != 1) && (special_screen != 2)) {
                if (time % 60 >= 10) {
                    special_screen = 2 + (time % 60) / 10;
                } else
                    special_screen = 0;
            }

            screen_print_col(s, L0, Black);
            ssd1306_SetCursor(90, L0);
            ssd1306_WriteString(strs[2 * special_screen], Font_7x10, White);
            ssd1306_SetCursor(90, L0 + 8);
            ssd1306_WriteString(strs[2* special_screen + 1], Font_7x10, White);
        }


        if ((data.temperature_degC != temp) || (data.humidity_perc != humid)) {
            temp = data.temperature_degC;
            humid = data.humidity_perc;

            float show_temp = temp;
            float show_humid = data.humidity_perc;
            float show_press = data_p.pressure_hPa;

            if (temp > max_temp)
                max_temp = temp;
            if (temp < min_temp)
                min_temp = temp;

            if (show_humid > max_humid)
                max_humid = show_humid;
            if (show_humid < min_humid)
                min_humid = show_humid;

            if (show_press > max_press)
                max_press = show_press;
            if (show_press < min_press)
                min_press = show_press;

            char temp_char = 't';

            if (special_screen == 1) {
                show_temp = max_temp;
                show_humid = max_humid;
                show_press = max_press;
                temp_char = 'M';
            }
            if (special_screen == 2) {
                temp_char = 'm';
                show_temp = min_temp;
                show_humid = min_humid;
                show_press = min_press;
            }

            float temp_1 = data_p.temperature_degC;

            /* Debug only with sudo screen /dev/ttyACM0 115200
            printf("%02i:%02i:%02i : t_degC*100= %i %i; hmdt*10= %i; Pa= %i \r\n",
                (time / 60) / 60,
                (time / 60) % 60, time % 60,
                (int)(temp * 100), (int)(temp_1 * 100),
                (int)(data.humidity_perc * 10),
                (int) (data_p.pressure_hPa * 10)
            ); */

            sprintf(s, "%c%+2i.%02i", temp_char,
                (int)(show_temp), ((int) (show_temp * 100)) % 100);
                //(int)(temp_1), ((int) (temp_1 * 10)) % 10);
            screen_print_col(s, L1, White);

            sprintf(s, "/%i.%01i",
                (int)(temp_1), ((int) (temp_1 * 10)) % 10);
            screen_print_col_small("o", L1 - 4, White);
            screen_print_col_small(" C", L1, White);
            screen_print_col_small(s, L1 + 9, White);

            sprintf(s, "%3i.%01i%%",
                (int)(show_humid),
                ((int)(show_humid * 10)) % 10);
            screen_print_col(s, L2, White);
            
            sprintf(s, "%4i.%02i",
                (int) (show_press),
                (int) (show_press * 100) % 100
            );
            screen_print_col_small(s, L2 + 4, White);

            // hPa
            {
                ssd1306_SetCursor(100, L2 + 16);
                // sprintf(s, "%i", shake_count);
                ssd1306_WriteString("hPa", Font_7x10, White);
            }
        }
        
        ssd1306_UpdateScreen();
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

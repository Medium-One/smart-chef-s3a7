/***********************************************************************************************************************
 * Copyright [2015] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
 *
 * The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
 * and/or its licensors ("Renesas") and subject to statutory and contractual protections.
 *
 * Unless otherwise expressly agreed in writing between Renesas and you: 1) you may not use, copy, modify, distribute,
 * display, or perform the contents; 2) you may not use any name or mark of Renesas for advertising or publicity
 * purposes or in connection with your use of the contents; 3) RENESAS MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE
 * SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED "AS IS" WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR CONSEQUENTIAL DAMAGES,
 * INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF CONTRACT OR TORT, ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents included in this file may
 * be subject to different terms.
 **********************************************************************************************************************/

 /*******************************************************************************
 * File Name    : vibration_detection_thread.c
 * Version      : 1.0
 * Device(s)    : S3A7
 * Tool-Chain   : e2studio, GNU GCC 4.9
 * OS           : ThreadX
 * H/W Platform : S3A7 IoT Enabler
 * Description  : Vibration Detection thread samples the accelerometer once
 *                every 10 ms and performs some basic aggregation, then sends
 *                the aggregated data to the cloud every sample_period.
 ******************************************************************************/

#include <app.h>
#include "vibration_detection_thread.h"
#include <m1_agent.h>

#include <stdio.h>

float mag_calc(float x, float y, float z);
float q_sqrt(float x);
void vibration_detection_thread_entry(void);

#define USE_SHARED_BUS
#define SLEEP_STEP 1

int sample_period = 6000;

/******************************************************************************
* Function Name: q_sqrt
* Description  : Fast square-root calculation using Greg Walsh's method.
* Arguments    : x –
*                    a binary32 floating point value.
* Return Value : The approximate square-root of x.
******************************************************************************/
float q_sqrt(float x) {
    int * i = (int *)&x;

    *i = 0x1fbd1df5 + (*i >> 1);
    return *((float *)i);
}

/******************************************************************************
* Function Name: mag_calc
* Description  : Calculates magnitude of euclidean vector given by x, y, z
* Arguments    : x –
*                    the x component of the vector.
* Arguments    : y –
*                    the y component of the vector.
* Arguments    : z –
*                    the z component of the vector.
* Return Value : The approximate magnitude.
******************************************************************************/
float mag_calc(float x, float y, float z) {
    return q_sqrt(x * x + y * y + z * z);
}

volatile bool send_connect_event = true;

/******************************************************************************
* Function Name: vibration_detection_thread_entry
* Description  : Thread begins execution after being resumed by net_thread,
*                after successful connection to the cloud. Configures the
*                accelerometer. Acceletrometer can be PmodACL2 or
*                BMC150 (#define BMC150). If BMC150 and I2C_VIBRATION is
*                defined, uses the pre-configured I2C device g_sf_i2c_device4.
*                Otherwise uses the pre-configured SPI device g_sf_spi_device0.
*                Infinitely samples x, y, and z acceleration values and builds
*                the following aggregates for each over sample_period time:
*                    - min
*                    - max
*                    - average
*                    - number of zero (avergae) crossings in the last period
*                Aggregates are sent to the cloud every sample_period. Also
*                calculates min, max, and average acceleration magnitude, but
*                does not send to the cloud.
******************************************************************************/
void vibration_detection_thread_entry(void)
{
    char buf[20];
    char eventbuf[500] = {0};
    int sleep_count = 0;
    ssp_err_t err;

    uint8_t first_flag = 0;
    uint16_t sample_cnt = 0;
    uint8_t x_zero_cross = 0;
    uint8_t y_zero_cross = 0;
    uint8_t z_zero_cross = 0;
    float x_prev_avg = 0;
    float y_prev_avg = 0;
    float z_prev_avg = 0;
    float x_last = 0;
    float y_last = 0;
    float z_last = 0;
    float mag_max = -1000000;
    float mag_min = 1000000;
    float mag_tot = 0;
    float x_max = -1000000;
    float x_min = 1000000;
    float x_tot = 0;
    float y_max = -1000000;
    float y_min = 1000000;
    float y_tot = 0;
    float z_max = -1000000;
    float z_min = 1000000;
    float z_tot = 0;


#ifdef BMC150
#ifdef I2C_VIBRATION
    //read acceleration
#ifdef USE_SHARED_BUS
    err = g_sf_i2c_device4.p_api->open(g_sf_i2c_device4.p_ctrl, g_sf_i2c_device4.p_cfg);
    buf[0] = 0x34;
    buf[1] = 0x04;
    g_sf_i2c_device4.p_api->write(g_sf_i2c_device4.p_ctrl, buf, 2, false, 100);
#else
    err = g_i2c1.p_api->open(g_i2c1.p_ctrl, g_i2c1.p_cfg);
    err = g_i2c1.p_api->reset(g_i2c1.p_ctrl);
#endif
    APP_ERR_TRAP(err);
#else
    //read acceleration
    err = g_sf_spi_device0.p_api->open(g_sf_spi_device0.p_ctrl, g_sf_spi_device0.p_cfg);
    APP_ERR_TRAP(err);
#endif
#else
    // init pmodacl2
    err = g_sf_spi_device0.p_api->open(g_sf_spi_device0.p_ctrl, g_sf_spi_device0.p_cfg);
    APP_ERR_TRAP(err);
    buf[0] = 0x0A;
    buf[1] = 0x1F;
    buf[2] = 0x52;
    tx_thread_sleep(10);
    err = g_sf_spi_device0.p_api->writeRead(g_sf_spi_device0.p_ctrl, buf, &buf[8], 3, SPI_BIT_WIDTH_8_BITS, TX_WAIT_FOREVER);
    buf[1] = 0x2c;
    buf[2] = 0x93;
    err = g_sf_spi_device0.p_api->writeRead(g_sf_spi_device0.p_ctrl, buf, &buf[8], 3, SPI_BIT_WIDTH_8_BITS, TX_WAIT_FOREVER);
    buf[1] = 0x2d;
    buf[2] = 0x02;
    err = g_sf_spi_device0.p_api->writeRead(g_sf_spi_device0.p_ctrl, buf, &buf[8], 3, SPI_BIT_WIDTH_8_BITS, TX_WAIT_FOREVER);
    err = g_sf_spi_device0.p_api->close(g_sf_spi_device0.p_ctrl);
#endif

    while (1) {
        if (sleep_count > sample_period) {
            sleep_count = 0;
            if (first_flag) {
                x_prev_avg = x_tot / sample_cnt;
                y_prev_avg = y_tot / sample_cnt;
                z_prev_avg = z_tot / sample_cnt;
                sprintf(eventbuf, "{"
                        "\"x_max\":%f,"
                        "\"x_min\":%f,"
                        "\"x_avg\":%f,"
                        "\"y_max\":%f,"
                        "\"y_min\":%f,"
                        "\"y_avg\":%f,"
                        "\"z_max\":%f,"
                        "\"z_min\":%f,"
                        "\"z_avg\":%f,"
                        "\"sample_cnt\":%u,"
                        "\"x_zero_cross\":%u,"
                        "\"y_zero_cross\":%u,"
                        "\"z_zero_cross\":%u,"
                        "}"
                        ,
                        x_max,
                        x_min,
                        x_tot / sample_cnt,
                        y_max,
                        y_min,
                        y_tot / sample_cnt,
                        z_max,
                        z_min,
                        z_tot / sample_cnt,
                        sample_cnt,
                        x_zero_cross,
                        y_zero_cross,
                        z_zero_cross
                        );
                m1_publish_event(eventbuf, NULL);
                sample_cnt = 0;
                x_zero_cross = 0;
                y_zero_cross = 0;
                z_zero_cross = 0;
                mag_max = 0;
                mag_min = 1000000;
                mag_tot = 0;
                x_max = -1000000;
                x_min = 1000000;
                x_tot = 0;
                y_max = -1000000;
                y_min = 1000000;
                y_tot = 0;
                z_max = -1000000;
                z_min = 1000000;
                z_tot = 0;
                first_flag = 0;
            }
        }
#ifdef BMC150
#ifdef I2C_VIBRATION
        buf[0] = 0x02;
#ifdef USE_SHARED_BUS
        err = g_sf_i2c_device4.p_api->write(g_sf_i2c_device4.p_ctrl, buf, 1, true, 100);
#else
        err = g_i2c1.p_api->write(g_i2c1.p_ctrl, buf, 1, false);
#endif
        while (err != SSP_SUCCESS) {
#ifdef USE_SHARED_BUS
            err = g_sf_i2c_device4.p_api->write(g_sf_i2c_device4.p_ctrl, buf, 1, true, 100);
#else
            err = g_i2c1.p_api->write(g_i2c1.p_ctrl, buf, 1, false);
#endif
        }
#ifdef USE_SHARED_BUS
        err = g_sf_i2c_device4.p_api->read(g_sf_i2c_device4.p_ctrl, &buf[8], 6, false, 10);
#else
        err = g_i2c1.p_api->read(g_i2c1.p_ctrl, &buf[8], 6, false);
#endif
#else
        buf[0] = (char)(0x80 | 0x02);
        err = g_sf_spi_device0.p_api->writeRead(g_sf_spi_device0.p_ctrl, buf, &buf[7], 7, SPI_BIT_WIDTH_8_BITS, TX_WAIT_FOREVER);
#endif
#else
        buf[0] = 0x0B;
        buf[1] = 0x0e;
        err = g_sf_spi_device0.p_api->writeRead(g_sf_spi_device0.p_ctrl, buf, &buf[8], 8, SPI_BIT_WIDTH_8_BITS, TX_WAIT_FOREVER);
#endif
        if (err == SSP_SUCCESS) {
#ifdef BMC150
            int16_t xaccel = (int16_t)(((buf[8] >> 4) & 0x0f) | (((int16_t)buf[9]) << 4));
            int16_t yaccel = (int16_t)(((buf[10] >> 4) & 0x0f) | (((int16_t)buf[11]) << 4));
            int16_t zaccel = (int16_t)(((buf[12] >> 4) & 0x0f) | (((int16_t)buf[13]) << 4));

            float fXAccel = xaccel * 0.00098f;
            float fYAccel = yaccel * 0.00098f;
            float fZAccel = zaccel * 0.00098f;
#else
            int16_t xAccel = buf[10] | (buf[11] << 8);
            int16_t yAccel = buf[12] | (buf[13] << 8);
            int16_t zAccel = buf[14] | (buf[15] << 8);

            float dScale = 0.004;

            float fXAccel = xAccel * dScale;
            float fYAccel = yAccel * dScale;
            float fZAccel = zAccel * dScale;
#endif
            float mag_accel = mag_calc(fXAccel, fYAccel, fZAccel);
            if (mag_accel > mag_max) {
                mag_max = mag_accel;
            }
            if (mag_accel < mag_min) {
                mag_min = mag_accel;
            }
            if (fXAccel > x_max) {
                x_max = fXAccel;
            }
            if (fXAccel < x_min) {
                x_min = fXAccel;
            }
            x_tot += fXAccel;
            if (fYAccel > y_max) {
                y_max = fYAccel;
            }
            if (fYAccel < y_min) {
                y_min = fYAccel;
            }
            y_tot += fYAccel;
            if (fZAccel > z_max) {
                z_max = fZAccel;
            }
            if (fZAccel < z_min) {
                z_min = fZAccel;
            }
            z_tot += fZAccel;
            if ((x_last < x_prev_avg && fXAccel > x_prev_avg) || (x_last > x_prev_avg && fXAccel < x_prev_avg)) {
                x_zero_cross++;
            }
            if ((y_last < y_prev_avg && fYAccel > y_prev_avg) || (y_last > y_prev_avg && fYAccel < y_prev_avg)) {
                y_zero_cross++;
            }
            if ((z_last < z_prev_avg && fZAccel > z_prev_avg) || (z_last > z_prev_avg && fZAccel < z_prev_avg)) {
                z_zero_cross++;
            }
            mag_tot += mag_accel;
            sample_cnt++;
            first_flag = 1;
            x_last = fXAccel;
            y_last = fYAccel;
            z_last = fZAccel;
        }
        tx_thread_sleep(SLEEP_STEP);
        sleep_count += SLEEP_STEP;
    }
}

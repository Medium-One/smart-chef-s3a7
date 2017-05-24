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
 * File Name    : sensor_thread_entry.c
 * Version      : 1.0
 * Device(s)    : S3A7
 * Tool-Chain   : e2studio, GNU GCC 4.9
 * OS           : ThreadX
 * H/W Platform : S3A7 IoT Enabler
 * Description  : Sensor thread is responsible for peripheral block
 *                initialization and processing cloud driver commands.
 ******************************************************************************/

#include <app.h>
#include "sensor_thread.h"
#include "lcd_display_api.h"
#include <m1_agent.h>
#include <m1_cloud_driver.h>

#include <stdio.h>

void m1_message_callback(int type, char * topic, char * payload, int length);
void sensor_thread_entry(void);

extern int sample_period;
#ifdef I2C_MULTI_THREAD
extern TX_QUEUE g_i2c0_queue;
extern TX_QUEUE g_i2c1_queue;
#endif

/******************************************************************************
* Function Name: sensor_thread_entry
* Description  : Initializes the comms framework (UART) and pre-configured
*                I2C devices. Infinitely waits on tx queue for cloud driver
*                messages and processes them using the M1 Synergy Cloud Driver
*                library, freeing the message buffer afterward.
******************************************************************************/
void sensor_thread_entry(void)
{
    ssp_err_t err;
    char * cloud_driver_command;

    // open PMOD/Grove B as UART(?)
#if 0
    err = g_sf_comms0.p_api->open(g_sf_comms0.p_ctrl, g_sf_comms0.p_cfg);
    APP_ERR_TRAP(err);
#endif
    m1_initialize_comms(&g_sf_comms0);

    // open PMOD A as I2C (known devices)
#ifdef I2C_OPEN
    err = g_sf_i2c_device0.p_api->open(g_sf_i2c_device0.p_ctrl, g_sf_i2c_device0.p_cfg);
    APP_ERR_TRAP(err);
    m1_initialize_i2c(&g_sf_i2c_device0);
#ifndef I2C_MULTI_THREAD
    err = g_sf_i2c_device1.p_api->open(g_sf_i2c_device1.p_ctrl, g_sf_i2c_device1.p_cfg);
    APP_ERR_TRAP(err);
    m1_initialize_i2c(&g_sf_i2c_device1);
    err = g_sf_i2c_device2.p_api->open(g_sf_i2c_device2.p_ctrl, g_sf_i2c_device2.p_cfg);
    APP_ERR_TRAP(err);
    m1_initialize_i2c(&g_sf_i2c_device2);
    err = g_sf_i2c_device3.p_api->open(g_sf_i2c_device3.p_ctrl, g_sf_i2c_device3.p_cfg);
    APP_ERR_TRAP(err);
    m1_initialize_i2c(&g_sf_i2c_device3);
#endif
#endif

    // we don't open PMOD C because it is used by the vibration thread

    // we don't open PMOD D because wifi doesn't share

    char rxBuf[700];

    while (1)
    {
        tx_queue_receive(&g_cloud_driver_command_queue, &cloud_driver_command, TX_WAIT_FOREVER);
        if (m1_handle_message(cloud_driver_command, rxBuf) == M1_SUCCESS_DATA)
            m1_publish_event(rxBuf, NULL);

        free(cloud_driver_command);
    }
}

extern const sf_message_instance_t g_sf_message0;

/******************************************************************************
* Function Name: m1_message_callback
* Description  : Callback routine to handle messages published to subscribed
*                topic. This routine is called by the M1 VSA thread.
*                3 different types of messages can be handled in this routine:
*                    1. Settings update. Messages starting with 'S' are
*                       interpreted as settings update, which can adjust the
*                       global int sample_period
*                       (see vibration_detection_thread).
*                    2. Display command. Messages starting with 'D' are
*                       interpreted as commands to display strings to the LCD.
*                       A message framework is used to post the string to the
*                       GUI thread.
*                    3. All other messages are assumed to be cloud driver
*                       commands. Buffers to hold the message are malloc'd and
*                       sent to the TX_QUEUE which the sensor thread is
*                       listening on.
* Arguments    : See M1 VSA documentation
******************************************************************************/
void m1_message_callback(int type, char * topic, char * payload, int length) {
    int ret;
    ssp_err_t err;
    int updated_sample_period;
    SSP_PARAMETER_NOT_USED(type);
    SSP_PARAMETER_NOT_USED(topic);

    switch (payload[0]) {
        case 'S': {
            // this is a settings update
            if (!strncmp(&payload[1], "vibration_window", (((size_t)length) < strlen("vibration_window")) ? (size_t)length : strlen("vibration_window"))) {
                ret = sscanf(&payload[1 + strlen("vibration_window")], "%d", &updated_sample_period);
                if (ret == 1)
                    sample_period = updated_sample_period / 10;
            }
            break;
        } case 'D': {
            // this is a display command
            unsigned int line_number;
            int n;
            ret = sscanf(&payload[1], "%u;%n", &line_number, &n);
            if (ret == 1) {
                lcd_display_payload_t * p_msg_header;
                sf_message_acquire_cfg_t acquire_cfg;
                sf_message_post_cfg_t post_cfg;
                acquire_cfg.buffer_keep = false;
                post_cfg.p_callback = NULL;
                post_cfg.priority = SF_MESSAGE_PRIORITY_NORMAL;
                sf_message_post_err_t post_err;
                do {
                    err = g_sf_message0.p_api->bufferAcquire(g_sf_message0.p_ctrl,
                                                   (sf_message_header_t **)&p_msg_header,
                                                   &acquire_cfg,
                                                   10);
                } while (err != SSP_SUCCESS);
                p_msg_header->header.event_b.class = SF_MESSAGE_EVENT_CLASS_LCD_DISPLAY;
                p_msg_header->header.event_b.code = SF_MESSAGE_EVENT_NEW_DATA;
                p_msg_header->line_number = line_number;
                strncpy(p_msg_header->msg, &payload[1] + n, 100);
                err = g_sf_message0.p_api->post(g_sf_message0.p_ctrl,
                                          (sf_message_header_t *)p_msg_header,
                                          &post_cfg,
                                          &post_err,
                                          10);
                APP_ERR_TRAP(err);
            }
            break;
        } default: {

            char * newBuf = (char *)malloc((size_t)length);
            if (newBuf != NULL) {
                memcpy(newBuf, payload, (size_t)length);
                err = tx_queue_send(&g_cloud_driver_command_queue, &newBuf, 20);
                if (err) {
                    if (err != TX_QUEUE_FULL)
                        APP_ERR_TRAP(err);
                    free(newBuf);
                }
            }
            break;
        }
    }
}

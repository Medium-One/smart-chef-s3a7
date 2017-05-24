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
 * File Name    : gui_thread_entry.c
 * Version      : 1.0
 * Device(s)    : S3A7
 * Tool-Chain   : e2studio, GNU GCC 4.9
 * OS           : ThreadX
 * H/W Platform : S3A7 IoT Enabler
 * Description  : GUI thread is responsible for initialization of the LCD and
 *                the initial display sequence of images. Subsequently it
 *                listens to the message framework g_sf_message0 for touch
 *                events and text display commands.
 ******************************************************************************/

#include <app.h>
#include "gui_thread.h"
#include <m1_agent.h>

#include <stdio.h>
#include <stdarg.h>

extern void ConfigureDisplayHardware565rgb (void);
extern void PaintScreen (uint8_t * p_data);

extern const uint16_t m1provision[];
extern int provisioning;

int printf(const char *fmt, ...);
void gui_thread_entry(void);

static sf_message_header_t * p_message = NULL;


/******************************************************************************
* Function Name: printf
* Description  : printf replacement which outputs to the LCD screen at row 0,
*                character 0.
* Arguments    : fmt –
*                    Format string.
*                varargs -
*                    See printf documentation.
* Return Value : See printf documentation.
******************************************************************************/
int printf(const char *fmt, ...)
{
    char buf[40];
    va_list argp;
    va_start(argp, fmt);
    int ret = vsnprintf(buf, 40, fmt, argp);
    va_end(argp);
    BufferLine(0, buf);
    PaintText();
    return ret;
}

/******************************************************************************
* Function Name: gui_thread_entry
* Description  : Initializes the LCD panel. Displays initial sequence of
*                images. Communicates provisioning mode selection to other
*                threads via global int `provisioning`. Blocks other threads
*                until provisioning mode selection is complete via
*                g_provision_lock events flag group. Listens infinitely to
*                the message framework g_sf_message0 for touch events and
*                text display commands.
******************************************************************************/
void gui_thread_entry(void)
{
    ssp_err_t err;
    char event[128];

    g_Blacklight_PWM.p_api->open(g_Blacklight_PWM.p_ctrl, g_Blacklight_PWM.p_cfg);
    g_Blacklight_PWM.p_api->dutyCycleSet(g_Blacklight_PWM.p_ctrl, 100, TIMER_PWM_UNIT_PERCENT, 0);

    ConfigureDisplayHardware565rgb();
    PaintScreen((uint8_t *) m1provision);
    // wait up to 10 seconds for user to enter provisioning mode
    // any touch event is considered to mean the user selected provisioning mode
    err = g_sf_message0.p_api->pend(g_sf_message0.p_ctrl, &gui_thread_message_queue, (sf_message_header_t **) &p_message, 1000);
    if (err == SSP_SUCCESS) {
        if ((SF_MESSAGE_EVENT_NEW_DATA == p_message->event_b.code) && (p_message->event_b.class == SF_MESSAGE_EVENT_CLASS_TOUCH)) {
            // drain any other messages
            do {
                err = g_sf_message0.p_api->pend(g_sf_message0.p_ctrl, &gui_thread_message_queue, (sf_message_header_t **) &p_message, 30);
                if (err == SSP_SUCCESS)
                    g_sf_message0.p_api->bufferRelease(g_sf_message0.p_ctrl, (sf_message_header_t *) p_message, SF_MESSAGE_RELEASE_OPTION_NONE);
            } while (err != SSP_ERR_MESSAGE_QUEUE_EMPTY);
            provisioning = 1;
        }
    }

    tx_event_flags_set(&g_provision_lock, PROVISIONING_COMPLETED_FLAG, TX_OR);

    if (!provisioning) {
        while (1) {
            err = g_sf_message0.p_api->pend(g_sf_message0.p_ctrl, &gui_thread_message_queue, (sf_message_header_t **) &p_message, TX_WAIT_FOREVER);

            switch (p_message->event_b.class)
            {
            case SF_MESSAGE_EVENT_CLASS_TOUCH:
                if (SF_MESSAGE_EVENT_NEW_DATA == p_message->event_b.code) {
                    sf_touch_panel_payload_t * p_touch_message = (sf_touch_panel_payload_t *) p_message;
                    if (p_touch_message->event_type == SF_TOUCH_PANEL_EVENT_UP) {
                        sprintf(event, "{\"touched\":true,\"x\":%d,\"y\":%d}", p_touch_message->x, p_touch_message->y);
                        m1_publish_event(event, NULL);
                    }
                }
                break;
            case SF_MESSAGE_EVENT_CLASS_LCD_DISPLAY:
                if (SF_MESSAGE_EVENT_NEW_DATA == p_message->event_b.code) {
                    lcd_display_payload_t * p_display_message = (lcd_display_payload_t *) p_message;
                    BufferLine((int)p_display_message->line_number, p_display_message->msg);
                    PaintText();
                }
                break;
            default:
                break;
            }
            /** Message is processed, so release buffer. */
            err = g_sf_message0.p_api->bufferRelease(g_sf_message0.p_ctrl, p_message, SF_MESSAGE_RELEASE_OPTION_NONE);
            if (err)
            {
                while(1);
            }
        }
    }
}

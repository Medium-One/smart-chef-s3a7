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
 * File Name    : synergy_grapics_driver_565rgb.h
 * Version      : 1.0
 * Device(s)    : S3A7
 * Tool-Chain   : e2studio, GNU GCC 4.9
 * OS           : ThreadX
 * H/W Platform : S3A7 IoT Enabler
 * Description  : Graphics LCD driver header file.
 ******************************************************************************/

#ifndef LCD_SYNERGY_GRAPHICS_DRIVER_565RGB_H_
#define LCD_SYNERGY_GRAPHICS_DRIVER_565RGB_H_

#include <stdint.h>

void PaintScreen (uint8_t * p_data);
int BufferLine(int line, char * str);
void PaintText();
void ConfigureDisplayHardware565rgb (void);

#endif /* LCD_SYNERGY_GRAPHICS_DRIVER_565RGB_H_ */

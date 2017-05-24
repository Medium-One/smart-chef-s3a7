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

#ifndef M1_H_
#define M1_H_

char fatalErrString[41];

//#define M1_DEBUG
#define USE_UMQTT
//#define M1_USE_MALLOC
#define APPLICATION_NAME "M1"
#define USB_PROVISION

#define I2C_OPEN
//#define I2C_LOCKING
#define I2C_RESTART
//#define I2C_MULTI_THREAD
//#define I2C_DEBUG
#define I2C_DEBUG2


#define BMC150
//#define I2C_VIBRATION

//#define ENABLE_USB

#include "synergy_graphics_driver_565rgb.h"


#ifdef M1_DEBUG
#define APP_ERR_TRAP(a)     do {\
		                        if(a) {\
                                    snprintf(fatalErrString, 41, "%.30s:%d:%u", __FILE__, __LINE__, a);\
                                    BufferLine(0, fatalErrString);\
                                    PaintText();\
                                    __asm("BKPT");\
		                        }\
                            } while (0)
#else
#define APP_ERR_TRAP(a)     do {\
                                if(a) {\
                                    BufferLine(0, "#### SENSOR ERROR, PLEASE RESTART ####");\
                                    PaintText();\
                                }\
                            } while (0)
#endif

#define PROVISIONING_COMPLETED_FLAG 0x01

#endif /* M1_H_ */

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
 * File Name    : usb_device_thread_entry.c
 * Version      : 1.0
 * Device(s)    : S3A7
 * Tool-Chain   : e2studio, GNU GCC 4.9
 * OS           : ThreadX
 * H/W Platform : S3A7 IoT Enabler
 * Description  : USB device thread is responsible for initializating the FX
 *                and UX systems, configuring the stack to operate in device
 *                mode, and writing the provisioning information to data flash
 *                when the provisioning text file is created.
 ******************************************************************************/

#include <app.h>
#include "usb_device_thread.h"
#include "ux_api.h"
#include "ux_dcd_synergy.h"
#include "ux_device_class_storage.h"
#include "fx_api.h"
#include "net_thread.h"

#define RAM_DISK_LAST_LBA ((13*1024 / 512) - 1)


void usb_device_thread_entry(void);

extern TX_THREAD usb_device_thread;

#define USB_HOST_ENABLE IOPORT_PORT_07_PIN_04
#define DATA_FLASH_BLOCK_SIZE 64
#define DATA_FLASH_PROGRAMMING_UNIT 4

UINT usb_ramdisk_media_read   (void * storage, ULONG lun, UCHAR * p_dest,
                               ULONG blocks, ULONG lba, ULONG * status);
UINT usb_ramdisk_media_write  (void * storage, ULONG lun, UCHAR * p_src,
                               ULONG blocks, ULONG lba, ULONG * status);
UINT usb_ramdisk_media_status (void * storage, ULONG lun, ULONG id, ULONG * status);


#ifdef ENABLE_USB
static UCHAR usb_memory[12*1024] __attribute__ ((aligned(4)));
#endif

extern const UCHAR   device_usbfs[50];
extern const UCHAR   device_usbhs[60];
extern const UCHAR * device_strings;
extern const UCHAR * device_lang;

UCHAR          ram_disk_memory[13*1024] __attribute__ ((aligned(4)));
UCHAR          ram_disk_sector_cache[512];
FX_MEDIA       ram_disk_media;
extern VOID    _fx_ram_driver(FX_MEDIA *media_ptr);

extern int provisioning;
extern TX_EVENT_FLAGS_GROUP g_provision_lock;

UCHAR * gp_partition = ram_disk_memory;

/******************************************************************************
* Function Name: usb_device_thread_entry
* Description  : Initializes the FX system first. Waits for g_provision_lock
*                events flag group to indicate that WiFi provisioning was not
*                selected, then initializes the UX system and configures the
*                USB stack to operate in device mode. Infinitely waits for
*                the provisioning text file to be created, then writes the
*                provisioning information to data flash. After writing
*                provisioning information to data flash, infinitely sleeps.
*                If the provisioning text file does not contain the required
*                content, the information is not written to data flash.
******************************************************************************/
void usb_device_thread_entry(void)
{
    UINT status;

    /* Initialize FileX.  */
    fx_system_initialize();


    /* Format the RAM disk - the memory for the RAM disk was defined above.  */
    status = fx_media_format(&ram_disk_media,
                          _fx_ram_driver,                  /* Driver entry             */
                          ram_disk_memory,                 /* RAM disk memory pointer  */
                          ram_disk_sector_cache,           /* Media buffer pointer     */
                          sizeof(ram_disk_sector_cache),   /* Media buffer size        */
                          (CHAR*)"MY_RAM_DISK",            /* Volume Name              */
                          1,                               /* Number of FATs           */
                          32,                              /* Directory Entries        */
                          0,                               /* Hidden sectors           */
                          sizeof(ram_disk_memory) / sizeof(ram_disk_sector_cache),/* Total sectors            */
                          sizeof(ram_disk_sector_cache),   /* Sector size              */
                          1,                               /* Sectors per cluster      */
                          1,                               /* Heads                    */
                          1);                              /* Sectors per track        */

    if (status)
       while (1);

    status = fx_media_open(&ram_disk_media, (CHAR*)"RAM DISK", _fx_ram_driver, ram_disk_memory, ram_disk_sector_cache, sizeof(ram_disk_sector_cache));
    if (status)
       while (1);

    ULONG actual_flags;
    tx_event_flags_get(&g_provision_lock, PROVISIONING_COMPLETED_FLAG, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

    if (provisioning)
        tx_thread_suspend(&usb_device_thread);

#ifdef ENABLE_USB
    g_ioport.p_api->pinWrite(USB_HOST_ENABLE,IOPORT_LEVEL_LOW);    // High selects the USB HOST connector


    ux_system_initialize(usb_memory, sizeof(usb_memory), UX_NULL, 0);

     status = _ux_device_stack_initialize((UCHAR *) device_usbhs,   sizeof(device_usbhs),
                                          (UCHAR *) device_usbfs,   sizeof(device_usbfs),
                                          (UCHAR *) device_strings, strlen((const void *) device_strings),
                                          (UCHAR *) device_lang,    strlen((const void *) device_lang),
                                          UX_NULL);
     if (status)
         while (1);

     g_storage.ux_slave_class_storage_parameter_number_lun = 1;

     gp_lun_0->ux_slave_class_storage_media_last_lba       = RAM_DISK_LAST_LBA;
     gp_lun_0->ux_slave_class_storage_media_block_length   = 512;
     gp_lun_0->ux_slave_class_storage_media_type           = 0;
     gp_lun_0->ux_slave_class_storage_media_removable_flag = 0x80;
     gp_lun_0->ux_slave_class_storage_media_read           = usb_ramdisk_media_read;
     gp_lun_0->ux_slave_class_storage_media_write          = usb_ramdisk_media_write;
     gp_lun_0->ux_slave_class_storage_media_status         = usb_ramdisk_media_status;

     status = ux_device_stack_class_register(_ux_system_slave_class_storage_name,
                                             _ux_device_class_storage_entry,
                                             1, 0, (void *) &g_storage);
     if (status)
         while (1);

     _ux_dcd_synergy_initialize((ULONG) R_USBFS);

     while (1)
     {
         FX_FILE file;
         char buffer[300];
         ULONG bytes_read;
/*
 *      Filex reads are cached - must invalidate cache to see what the host wrote
 */
         fx_media_cache_invalidate(&ram_disk_media);
         status = fx_file_open(&ram_disk_media, &file, "provision.txt", FX_OPEN_FOR_READ);

         if (status)
             tx_thread_sleep(10);
         else
         {

             fx_file_read(&file, buffer, sizeof(buffer) - 1, &bytes_read);
             fx_file_close(&file);
             buffer[bytes_read] = '\0';
             // ensure all required keys are here
             if (strstr("ssid=", buffer) ||
                     strstr("key=", buffer) ||
                     strstr("apikey=", buffer) ||
                     strstr("mqttuserid=", buffer) ||
                     strstr("mqttprojectid=", buffer) ||
                     strstr("password=", buffer) ||
                     strstr("sec=", buffer)) {
                 tx_thread_sleep(100);
                 continue;
             }
             status = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);

             uint32_t blocks = 0;
             long unsigned int temp_length = bytes_read + 1;
             while (temp_length > 0) {
                 blocks++;
                 temp_length -= DATA_FLASH_BLOCK_SIZE;
             }
             status = g_flash0.p_api->erase(g_flash0.p_ctrl, 0x40100000, blocks);
             status = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)buffer, 0x40100000,
                                             (bytes_read + DATA_FLASH_PROGRAMMING_UNIT) & (UINT)(~(DATA_FLASH_PROGRAMMING_UNIT - 1)));
             status = g_flash0.p_api->close(g_flash0.p_ctrl);
#endif
             while(1)
                 tx_thread_sleep(100);
#ifdef ENABLE_USB
         }
     }
#endif
 }


 UINT usb_ramdisk_media_read   (void * storage, ULONG lun, UCHAR * p_dest,
                                ULONG blocks, ULONG lba, ULONG * status)
 {
     SSP_PARAMETER_NOT_USED(storage);
     SSP_PARAMETER_NOT_USED(lun);
     SSP_PARAMETER_NOT_USED(status);

     memcpy(p_dest, (gp_partition + (512 * lba)), (512 * blocks));

     return (UX_SUCCESS);
 }

 UINT usb_ramdisk_media_write  (void * storage, ULONG lun, UCHAR * p_src,
                                ULONG blocks, ULONG lba, ULONG * status)
 {
     SSP_PARAMETER_NOT_USED(storage);
     SSP_PARAMETER_NOT_USED(lun);
     SSP_PARAMETER_NOT_USED(status);

     memcpy((gp_partition + (512 * lba)), p_src, (512 * blocks));

     return (UX_SUCCESS);
 }

 UINT usb_ramdisk_media_status (void * storage, ULONG lun, ULONG id, ULONG * status)
 {
     SSP_PARAMETER_NOT_USED(storage);
     SSP_PARAMETER_NOT_USED(lun);
     SSP_PARAMETER_NOT_USED(id);
     SSP_PARAMETER_NOT_USED(status);

     return (UX_SUCCESS);
 }

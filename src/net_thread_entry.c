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
 * File Name    : net_thread_entry.c
 * Version      : 1.0
 * Device(s)    : S3A7
 * Tool-Chain   : e2studio, GNU GCC 4.9
 * OS           : ThreadX
 * H/W Platform : S3A7 IoT Enabler
 * Description  : Net thread is responsible for network interface
 *                initialization, provisioning, and connecting to the cloud.
 *                Different LCD screens are displayed as the thread progresses.
 ******************************************************************************/

#include <app.h>
#include "net_thread.h"
#include "vibration_detection_thread.h"
#include "r_fmi.h"
#include "sf_wifi_api.h"
#include "fx_api.h"
#include "nx_api.h"
#include "nx_http_server.h"
#include "nx_dhcp.h"
#include "nx_dhcp_server.h"
#include "nx_dns.h"

#include <m1_agent.h>

#include <stdlib.h>

#define APP_SERVER_ADDRESS  (IP_ADDRESS(192,168,3,1))
#define APP_SERVER_MASK     (IP_ADDRESS(255,255,255,  0))
#define START_IP_ADDRESS_LIST       IP_ADDRESS(192, 168, 3, 100)
#define END_IP_ADDRESS_LIST         IP_ADDRESS(192, 168, 3, 200)
#define GW_ADDR                     IP_ADDRESS(192, 168, 3, 1 )

#define APP_USB_INS_FLAG    (0x1UL)
#define APP_USB_REM_FLAG    (0x2UL)

#define DATA_FLASH_BLOCK_SIZE 64
#define DATA_FLASH_PROGRAMMING_UNIT 4

void net_thread_entry(void);

extern TX_THREAD net_thread;
extern TX_THREAD vibration_detection_thread;
extern TX_EVENT_FLAGS_GROUP g_provision_lock;
extern const fmi_instance_t g_fmi0;

extern const uint16_t m1provisionend[];
extern const uint8_t provisionHtml[];

VOID nx_ether_driver_eth1(NX_IP_DRIVER *);
void m1_message_callback(int type, char * topic, char * payload, int length);
VOID g_sf_wifi_nsal_nx0(NX_IP_DRIVER * p_driver);
int qcom_set_tx_power(uint8_t device_id, uint32_t dbm);

extern FX_MEDIA ram_disk_media;
FX_MEDIA                    * gp_media = &ram_disk_media;
NX_PACKET_POOL  g_http_packet_pool;
NX_IP           g_http_ip;
static NX_HTTP_SERVER  g_http_server;

static NX_DHCP_SERVER   dhcp_server;
static NX_DHCP          g_dhcp;
NX_DNS           g_dns_client;

#define BLOCK_SIZE      (1536)
#ifdef USB_PROVISION
#define NUM_PACKETS     (5)
#define NUM_IP_PACKETS (10)
#define IP_THREAD_SIZE  (8 * 1024)
#else
#define NUM_PACKETS     (5)
#define NUM_IP_PACKETS (10)
#define IP_THREAD_SIZE  (10 * 1024)
#endif
#define DHCP_STACK_SIZE  (2 * 1024)
#define HTTP_STACK_SIZE (4 * 1024)


static CHAR mem_packet_pool[(BLOCK_SIZE + 32 + sizeof(NX_PACKET)) * NUM_IP_PACKETS] __attribute__ ((aligned(4)));
static CHAR mem_ip_stack[IP_THREAD_SIZE] __attribute__ ((aligned(4)));
static CHAR mem_arp[768] __attribute__ ((aligned(4)));
static CHAR mem_http_stack[HTTP_STACK_SIZE]  __attribute__ ((aligned(4)));
static CHAR dhcp_server_stack [DHCP_STACK_SIZE];
static char dhcp_buffer_pool_memory [BLOCK_SIZE*NUM_PACKETS];

NX_PACKET_POOL dhcp_packet_pool;
NX_TCP_SOCKET g_client_socket;

sf_wifi_provisioning_t prov;

#define MQTT_USERNAME_LEN_MAX 128
#define MQTT_PASSWORD_LEN_MAX 128

char m1_apikey[65];
char m1_password[65];
char m1_mqtt_user_id[32];
char m1_mqtt_project_id[32];
char m1_device_id[33] = "s3a7";

static ioport_port_pin_t leds[4] =
{
    IOPORT_PORT_07_PIN_00,  /* RED */
    IOPORT_PORT_07_PIN_01,  /* Yellow */
    IOPORT_PORT_07_PIN_02,  /* Green    */
    IOPORT_PORT_07_PIN_03   /* Green  */
};

UINT my_get_notify(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr);
ULONG actual_size;

int provisioning = 0;

#ifndef USB_PROVISION
UCHAR                   ram_disk_memory[13*1024] __attribute__ ((aligned(4)));
UCHAR                   ram_disk_sector_cache[512];
FX_MEDIA                ram_disk_media;
extern VOID    _fx_ram_driver(FX_MEDIA *media_ptr);
#endif

/******************************************************************************
* Function Name: urldecode
* Description  : Utility function to perform ASCII URL decoding
* Arguments    : buffer –
*                    string to decode.
*                length -
*                    legnth of string.
******************************************************************************/
static void urldecode(UCHAR * buffer, int length) {
    int status;

    UCHAR * urldecode = (UCHAR *)strchr((char *)buffer, '%');
    while (urldecode != NULL) {
        status = sscanf((char *)&urldecode[1], "%2hhx", urldecode);
        if (status != 1) {
            // TODO: return error to user
        }
        memcpy(&urldecode[1], &urldecode[3], (size_t)(length - (urldecode - buffer)));
        urldecode = (UCHAR *)strchr((char *)&urldecode[1], '%');
    }
}

/******************************************************************************
* Function Name: net_thread_entry
* Description  : Initializes the network interface, then enters either
*                provisioning or normal mode. In provisioning mode, launches
*                HTTP server and stores credentials to data-flash. In normal
*                mode, reads credentials from data-flash and connects to
*                the cloud using M1 VSA.
******************************************************************************/
void net_thread_entry(void) {
    UINT  status;
    ULONG actual_flags;
    ULONG ip_status, dhcp_status;
    ssp_err_t ssp_err;
    UINT addresses_added;
    UCHAR provisionConfigBuffer[300];

    nx_system_initialize();
    status = nx_packet_pool_create(&g_http_packet_pool,
                                   "HTTP Packet Pool",
                                   (BLOCK_SIZE + 32),
                                   mem_packet_pool,
                                   sizeof(mem_packet_pool));
    APP_ERR_TRAP(status);

    tx_event_flags_get(&g_provision_lock, PROVISIONING_COMPLETED_FLAG, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
    if (provisioning) {
        status = nx_packet_pool_create(&dhcp_packet_pool,
                                       "DHCP Server Pool",
                                       BLOCK_SIZE,
                                       dhcp_buffer_pool_memory,
                                       sizeof(dhcp_buffer_pool_memory));
        APP_ERR_TRAP(status);

        status = nx_ip_create(&g_http_ip, "HTTP IP Instance",
                              APP_SERVER_ADDRESS, APP_SERVER_MASK,
                              &g_http_packet_pool, g_sf_wifi_nsal_nx0,
                              mem_ip_stack, sizeof(mem_ip_stack), 2);
        APP_ERR_TRAP(status);
    } else {
        status = nx_ip_create(&g_http_ip, "HTTP IP Instance",
                              IP_ADDRESS(0,0,0,0), APP_SERVER_MASK,
                              &g_http_packet_pool, g_sf_wifi_nsal_nx0,
                              mem_ip_stack, sizeof(mem_ip_stack), 2);
        APP_ERR_TRAP(status);
    }
#ifndef I2C_DEBUG

    status = nx_ip_fragment_enable(&g_http_ip);
    APP_ERR_TRAP(status);

    status = nx_arp_enable(&g_http_ip, mem_arp, sizeof(mem_arp));
    APP_ERR_TRAP(status);

    status = nx_tcp_enable(&g_http_ip);
    APP_ERR_TRAP(status);

    status =  nx_udp_enable(&g_http_ip);
    APP_ERR_TRAP(status);

    status = nx_icmp_enable(&g_http_ip);
    APP_ERR_TRAP(status);

    /** Wait for init to finish. */
    status = nx_ip_interface_status_check(&g_http_ip, 0, NX_IP_LINK_ENABLED, &ip_status, NX_WAIT_FOREVER);
    APP_ERR_TRAP(status);

    if (provisioning) {
        status = nx_dhcp_server_create (&dhcp_server, &g_http_ip, dhcp_server_stack, sizeof(dhcp_server_stack),"DHCP Server", &dhcp_packet_pool);
        APP_ERR_TRAP(status);

        status = nx_dhcp_create_server_ip_address_list (&dhcp_server, 0, START_IP_ADDRESS_LIST, END_IP_ADDRESS_LIST, &addresses_added);
        APP_ERR_TRAP(status);

        status = nx_dhcp_set_interface_network_parameters (&dhcp_server, 0, APP_SERVER_MASK, GW_ADDR, GW_ADDR);
        APP_ERR_TRAP(status);
    } else {
        status = nx_dhcp_create(&g_dhcp, &g_http_ip, "Netx DHCP");
        APP_ERR_TRAP(status);

        status = nx_dns_create(&g_dns_client, &g_http_ip, (UCHAR *)"Netx DNS");
        APP_ERR_TRAP(status);
    }

    if (provisioning)  {
        status = nx_dhcp_server_start(&dhcp_server);
        APP_ERR_TRAP(status);
    } else {
        ssp_err = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
        APP_ERR_TRAP(ssp_err);
        ssp_err = g_flash0.p_api->read(g_flash0.p_ctrl, provisionConfigBuffer, 0x40100000, sizeof(provisionConfigBuffer) - 1);
        APP_ERR_TRAP(ssp_err);
        ssp_err = g_flash0.p_api->close(g_flash0.p_ctrl);
        APP_ERR_TRAP(ssp_err);
        actual_size = strnlen((char *)provisionConfigBuffer, sizeof(provisionConfigBuffer) - 1);
#if 0//def USB_PROVISION
        FX_FILE my_file;
        status = fx_file_create(gp_media, "provision.txt");
        APP_ERR_TRAP(status);
        status = fx_file_open(gp_media, &my_file, "provision.txt", FX_OPEN_FOR_WRITE);
        APP_ERR_TRAP(status);
        status = fx_file_write(&my_file, provisionConfigBuffer, actual_size);
        APP_ERR_TRAP(status);
        status = fx_file_close(&my_file);
        APP_ERR_TRAP(status);
        status = fx_media_flush(gp_media);
        APP_ERR_TRAP(status);
#endif
        char security[20];
        provisionConfigBuffer[actual_size] = '\0';
        char * temp, * dst, * delimiter;
        temp = (char *)provisionConfigBuffer;
        for (int i = 0; i < 7; i++) {
            if (!strncmp(temp, "ssid", 4))
                dst = (char *)prov.ssid;
            else if (!strncmp(temp, "key", 3))
                dst = (char *)prov.key;
            else if (!strncmp(temp, "apikey", 6))
                dst = m1_apikey;
            else if (!strncmp(temp, "mqttuserid", 10))
                dst = m1_mqtt_user_id;
            else if (!strncmp(temp, "mqttprojectid", 13))
                dst = m1_mqtt_project_id;
            else if (!strncmp(temp, "password", 8))
                dst = m1_password;
            else if (!strncmp(temp, "sec", 3))
                dst = security;
            else
                continue;
            temp = strchr(temp, '=') + 1;
            delimiter = strchr(temp, '&');
            if (delimiter == NULL)
                delimiter = strchr(temp, '\0');
            memcpy(dst, temp, (size_t)(delimiter - temp));
            dst[delimiter - temp] = '\0';
            urldecode((UCHAR *)dst, (int)strlen(dst));
            temp = delimiter + 1;
        }
        if (!strncmp(security, "open", 4))
            prov.security = SF_WIFI_SECURITY_TYPE_OPEN;
        else if (!strncmp(security, "wep", 9))
            prov.security = SF_WIFI_SECURITY_TYPE_WEP;
        else if (!strncmp(security, "wpa2", 4))
            prov.security = SF_WIFI_SECURITY_TYPE_WPA2;
        else if (!strncmp(security, "wpa", 3))
            prov.security = SF_WIFI_SECURITY_TYPE_WPA;
    }

    if (provisioning) {
#if 0

        uint8_t scan_count = 20;
        sf_wifi_scan_t scan_list[20];
        ssp_err = g_sf_wifi0.p_api->scan(g_sf_wifi0.p_ctrl, scan_list, &scan_count);
        tx_thread_sleep(500);
        scan_count = 20;
        ssp_err = g_sf_wifi0.p_api->scan(g_sf_wifi0.p_ctrl, scan_list, &scan_count);
#endif
        qcom_set_tx_power(0, 1);
        prov.security = SF_WIFI_SECURITY_TYPE_WPA2;
        prov.encryption = SF_WIFI_ENCRYPTION_TYPE_AUTO;
        prov.mode = SF_WIFI_INTERFACE_MODE_AP;
        fmi_product_info_t * p_fmi_product_info;
        g_fmi0.p_api->productInfoGet(&p_fmi_product_info);

        int seed = 0;
        for (unsigned int i = 0; i < sizeof(p_fmi_product_info->unique_id); i++)
            seed = p_fmi_product_info->unique_id[i] + (seed << 6) + (seed << 16) - seed;
        srand((unsigned int)seed);

        char supported_chars[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.";
        char supported_chars_password[] = "0123456789";
        char wifi_ssid[16] = "iot-wifi-";
        char wifi_password[11];
        // TODO: for more "security", skip a random number of sequence
        for (unsigned int j = strlen(wifi_ssid); j < sizeof(wifi_ssid) - 1; j++)
            wifi_ssid[j] = supported_chars[((unsigned int)rand()) % (sizeof(supported_chars) - 1)];
        wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
        for (unsigned int j = 0; j < sizeof(wifi_password) - 1; j++)
            wifi_password[j] = supported_chars_password[((unsigned int)rand()) % (sizeof(supported_chars_password) - 1)];
        wifi_password[sizeof(wifi_password) - 1] = '\0';
        strcpy((char *)&prov.ssid[0], wifi_ssid);
        strcpy((char *)&prov.key[0], wifi_password);
        prov.channel = !(rand() % 3) ? 1 : !(rand() % 2) ? 6 : 11;
    } else {
        prov.mode = SF_WIFI_INTERFACE_MODE_CLIENT;
        BufferLine(1, "Connecting to SSID:");
        BufferLine(2, (char *)prov.ssid);
        PaintText();
        switch (prov.security) {
            case SF_WIFI_SECURITY_TYPE_WPA2:
                prov.encryption = SF_WIFI_ENCRYPTION_TYPE_CCMP;
                break;
            case SF_WIFI_SECURITY_TYPE_WPA:
                prov.encryption = SF_WIFI_ENCRYPTION_TYPE_TKIP;
                break;
            case SF_WIFI_SECURITY_TYPE_WEP:
                prov.encryption = 3;
                break;
            default:
                prov.encryption = SF_WIFI_ENCRYPTION_TYPE_AUTO;
        }
    }

    do {
        ssp_err = g_sf_wifi0.p_api->provisioningSet(g_sf_wifi0.p_ctrl, &prov);
    } while (ssp_err != SSP_SUCCESS);
    APP_ERR_TRAP(ssp_err);


    if (!provisioning) {
        g_ioport.p_api->pinWrite(leds[2], IOPORT_LEVEL_HIGH);
        BufferLine(1, "Connected. Resolving IP address...");
        BufferLine(2, "");
        PaintText();

        status = nx_dhcp_start(&g_dhcp);
        APP_ERR_TRAP(status);

        status = nx_dns_server_add(&g_dns_client, (8L << 24) + (8L << 16) + (8L << 8) + 8L);
        APP_ERR_TRAP(status);

        nx_ip_status_check(&g_http_ip, NX_IP_ADDRESS_RESOLVED, &dhcp_status, NX_WAIT_FOREVER);
        BufferLine(1, "IP address resolved.");
        BufferLine(2, "Connecting to MQTT");
        PaintText();
    }

    while (1)
    {
        if (provisioning) {
#ifndef USB_PROVISION
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
            APP_ERR_TRAP(status);
            /* Open the RAM disk.  */
            status = fx_media_open(&ram_disk_media, (CHAR*)"RAM DISK", _fx_ram_driver, ram_disk_memory, ram_disk_sector_cache, sizeof(ram_disk_sector_cache));
            APP_ERR_TRAP(status);
            gp_media = &ram_disk_media;
#endif
            FX_FILE my_file;
            status = fx_file_create(gp_media, "index.html");
            APP_ERR_TRAP(status);
            status = fx_file_open(gp_media, &my_file, "index.html", FX_OPEN_FOR_WRITE);
            APP_ERR_TRAP(status);
            status = fx_file_write(&my_file, (void *)provisionHtml, strlen((char *)provisionHtml));
            APP_ERR_TRAP(status);
            status = fx_file_close(&my_file);
            APP_ERR_TRAP(status);

            status = nx_http_server_create(&g_http_server, "HTTP Server Instance", &g_http_ip,
                                           gp_media, mem_http_stack, sizeof(mem_http_stack),
                                           &g_http_packet_pool, NX_NULL, &my_get_notify);
            APP_ERR_TRAP(status);

            status = nx_http_server_start(&g_http_server);
            APP_ERR_TRAP(status);
            BufferLine(0, "Provisioning Mode");
            BufferLine(2, "Connect to the kit using");
            BufferLine(3, "SSID and key below. ");
            BufferLine(4, "Then go to ");
            BufferLine(5, "http://192.168.3.1 from ");
            BufferLine(6, "a browser.");
            sprintf((char *)provisionConfigBuffer, "SSID: %s", prov.ssid);
            BufferLine(8, (char *)provisionConfigBuffer);
            sprintf((char *)provisionConfigBuffer, "Key: %s", prov.key);
            BufferLine(9, (char *)provisionConfigBuffer);
            PaintText();
            tx_thread_suspend(&net_thread);
        } else {
            m1_register_subscription_callback(m1_message_callback);
            do {
                status = (UINT)m1_connect("mqtt2.mediumone.com",
                           61620,
                           m1_mqtt_user_id,
                           m1_password,
                           m1_mqtt_project_id,
                           m1_apikey,
                           "s3a7",
                           5,
                           5,
                           60,
                           1);
            } while (status != M1_SUCCESS);
            BufferLine(1, "Connected!");
            BufferLine(2, "");
            PaintText();
            m1_publish_event("{\"kit_version\":\"1.0.0\"}", NULL);
            tx_thread_resume(&vibration_detection_thread);
            tx_thread_suspend(&net_thread);
        }
        if (provisioning) {
            status = nx_http_server_stop(&g_http_server);
            APP_ERR_TRAP(status);

            status = nx_http_server_delete(&g_http_server);
            APP_ERR_TRAP(status);
        }

        g_ioport.p_api->pinWrite(leds[2], IOPORT_LEVEL_LOW);
    }
#else
    tx_thread_suspend(&net_thread);
#endif
}

/******************************************************************************
* Function Name: my_get_notify
* Description  : HTTP webserver request notify routine. This routine is called
*                prior to the HTTP server processing of the request.
*                Re-directs "/" to "/index.html" and writes provisioning form
*                data to data-flash.
* Arguments    : server_ptr –
*                    Pointer to HTTP Server control block.
*                request_type –
*                    HTTP Server request type
*                resource –
*                    resource portion of request URL
*                packet_ptr –
*                    Pointer to first packet buffer containing the request
* Return Value : NX_SUCCESS –
*                    Request was not a provisioning form submission.
*                NX_HTTP_CALLBACK_COMPLETED –
*                    Request was a provisioning form submission.
******************************************************************************/
UINT my_get_notify(NX_HTTP_SERVER *server_ptr, UINT request_type, CHAR *resource, NX_PACKET *packet_ptr)
{
    ssp_err_t ssp_err;
    int status;
    UINT length;
    NX_PACKET *response_pkt;
    UCHAR provisionConfigBuffer[300];

    /* Look for the test resource! */
    if ((request_type == NX_HTTP_SERVER_GET_REQUEST) && (strcmp(resource, "/") == 0)) {
        strcat(resource, "index.html");
    } else if(request_type == NX_HTTP_SERVER_POST_REQUEST) { /* Process multipart data. */
#if 0 // the entity stuff isn't working
        /* Get the content header. */
        while(nx_http_server_get_entity_header(server_ptr, &packet_ptr, provisionConfigBuffer, sizeof(provisionConfigBuffer)) == NX_SUCCESS) {
            /* Header obtained successfully. Get the content data location. */
            while(nx_http_server_get_entity_content(server_ptr, &packet_ptr, &offset, &length) == NX_SUCCESS) {
                /* Write content data to buffer. */
                nx_packet_data_extract_offset(packet_ptr, offset, provisionConfigBuffer, length, &length);
                provisionConfigBuffer[length] = 0;
            }
        }
#else
        status = (int)nx_http_server_content_get(server_ptr, packet_ptr, 0, (CHAR *)provisionConfigBuffer, sizeof(provisionConfigBuffer) - 1, &length);
#endif
        if (length >= sizeof(provisionConfigBuffer)) {

        }
        provisionConfigBuffer[length] = '\0';
        UCHAR * formdecode = (UCHAR *)strchr((char *)provisionConfigBuffer, '+');
        while (formdecode != NULL) {
            formdecode[0] = ' ';
            formdecode = (UCHAR *)strchr((char *)&formdecode[1], '+');
        }
        ssp_err = g_flash0.p_api->open(g_flash0.p_ctrl, g_flash0.p_cfg);
        APP_ERR_TRAP(ssp_err);

        uint32_t blocks = 0;
        int temp_length = (int)(length + 1);
        while (temp_length > 0) {
            blocks++;
            temp_length -= DATA_FLASH_BLOCK_SIZE;
        }
        ssp_err = g_flash0.p_api->erase(g_flash0.p_ctrl, 0x40100000, blocks);
        APP_ERR_TRAP(ssp_err);
        ssp_err = g_flash0.p_api->write(g_flash0.p_ctrl, (uint32_t)provisionConfigBuffer, 0x40100000,
                                        (length + DATA_FLASH_PROGRAMMING_UNIT) & (UINT)(~(DATA_FLASH_PROGRAMMING_UNIT - 1)));
        APP_ERR_TRAP(ssp_err);
        ssp_err = g_flash0.p_api->close(g_flash0.p_ctrl);
        APP_ERR_TRAP(ssp_err);

        PaintScreen((uint8_t *) m1provisionend);

        char * response = "<html><body>Provisioning complete. Please reset board to continue.</body></html>";
        if (length == sizeof(provisionConfigBuffer))
            status = (int)nx_http_server_callback_generate_response_header(server_ptr, &response_pkt, NX_HTTP_STATUS_ENTITY_TOO_LARGE, strlen(response), "text/html", "Server: NetX HTTP 5.3\r\n");
        else
            status = (int)nx_http_server_callback_generate_response_header(server_ptr, &response_pkt, NX_HTTP_STATUS_OK, strlen(response), "text/html", "Server: NetX HTTP 5.3\r\n");
        if (status == NX_SUCCESS) {
            status = (int)nx_packet_data_append(response_pkt, response, strlen(response), server_ptr->nx_http_server_packet_pool_ptr, NX_WAIT_FOREVER);
            if (status != NX_SUCCESS)
                nx_packet_release(response_pkt);
            else if (nx_http_server_callback_packet_send(server_ptr, response_pkt) != NX_SUCCESS) { // or nx_tcp_socket_send?
                nx_packet_release(response_pkt);
            }
        }
        /* Release the received client packet.      */
        nx_packet_release(packet_ptr);

        /* Indicate the response to client is transmitted. */
        return(NX_HTTP_CALLBACK_COMPLETED);
    }

    /* Indicate we have not processed the response to client yet.*/
    return NX_SUCCESS;
}

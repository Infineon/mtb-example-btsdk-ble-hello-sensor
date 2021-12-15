/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* BLE Vendor Specific Device
*
* This file provides definitions and function prototypes for Hello Sensor
* device
*
*/
#ifndef _HELLO_SENSOR_H_
#define _HELLO_SENSOR_H_

#include "wiced_hal_nvram.h"

#if BTSTACK_VER >= 0x03000001
#include "wiced_memory.h"
#include "bt_types.h"
#define BT_STACK_HEAP_SIZE          1024 * 6
#define wiced_bt_gatt_send_notification(id, type, len, ptr) wiced_bt_gatt_server_send_notification(id, type, len, ptr, NULL)
#define wiced_bt_gatt_send_indication(id, type, len, ptr)   wiced_bt_gatt_server_send_indication(id, type, len, ptr, NULL)
uint8_t *hello_sensor_alloc_buffer(uint16_t len);
void hello_sensor_free_buffer(uint8_t *p_data);
typedef void (*pfn_free_buffer_t)(uint8_t *);

#else // BTSTACK_VER

#include "wiced_gki.h"
#if !defined(CYW20735B1) && !defined(CYW20835B1) && !defined(CYW20719B1) && !defined(CYW20721B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2) && !defined(CYW20739B2)
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#endif

extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];
#endif // BTSTACK_VER

extern const wiced_transport_cfg_t transport_cfg;
void hello_sensor_gatt_init();
extern const uint8_t hello_sensor_gatt_database[];
extern size_t hello_sensor_gatt_database_size;

#ifndef PACKED
#define PACKED
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HELLO_SENSOR_MAX_NUM_CLIENTS 1

/* Hello Sensor App Timer Timeout in seconds  */
#define HELLO_SENSOR_APP_TIMEOUT_IN_SECONDS                 1

/* Hello Sensor App Fine Timer Timeout in milli seconds  */
#define HELLO_SENSOR_APP_FINE_TIMEOUT_IN_MS                 1

/* Hello Sensor Connection Idle  Timeout in milli seconds  */
#define HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS           3

#define HELLO_SENSOR_VS_ID                      WICED_NVRAM_VSID_START
#define HELLO_SENSOR_LOCAL_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 1 )
#define HELLO_SENSOR_PAIRED_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 2 )

#ifdef CYW20706A2
#define HELLO_SENSOR_GPIO_BUTTON_SETTINGS       WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE )
#define HELLO_SENSOR_GPIO_BUTTON_PRESSED_VALUE  WICED_GPIO_BUTTON_DEFAULT_STATE
#define HELLO_SENSOR_LED_GPIO        platform_led[hello_sensor_led_pin].led_gpio
#endif

#ifdef CYW20735B0
#define HELLO_SENSOR_GPIO_BUTTON_SETTINGS        WICED_GPIO_BUTTON_SETTINGS
#define HELLO_SENSOR_GPIO_BUTTON_PRESSED_VALUE   WICED_BUTTON_PRESSED_VALUE
#define HELLO_SENSOR_LED_GPIO       WICED_GPIO_PIN_LED1
#endif

#ifdef CYW20719B0
#define HELLO_SENSOR_GPIO_BUTTON_SETTINGS        WICED_GPIO_BUTTON_SETTINGS
#define HELLO_SENSOR_GPIO_BUTTON_PRESSED_VALUE   WICED_BUTTON_PRESSED_VALUE
#define HELLO_SENSOR_LED_GPIO       WICED_GPIO_PIN_LED1
#endif

#if defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20819A1) || defined(CYW20739B2)
#define HELLO_SENSOR_LED_GPIO        (uint32_t)*platform_led[hello_sensor_led_pin].gpio
#endif

#if defined(CYW55572) // We still cannot use Configurator to configure LED yet, we just force it to use PIN 26
 #define HELLO_SENSOR_LED_GPIO    26
#endif

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_HSENS_GATT_SERVICE = 0x1, // service handle

    HANDLE_HSENS_GAP_SERVICE = 0x14, // service handle
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, // characteristic handl
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL, // char value handle

        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, // characteristic handl
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,// char value handle


    HANDLE_HSENS_SERVICE = 0x28,
        HANDLE_HSENS_SERVICE_CHAR_NOTIFY, // characteristic handl
        HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, // char value handle
        HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, // charconfig desc handl

        HANDLE_HSENS_SERVICE_CHAR_BLINK, // characteristic handl
        HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL, // char value handle

        HANDLE_HSENS_SERVICE_CHAR_LONG_MSG, // characteristic handl
        HANDLE_HSENS_SERVICE_CHAR_LONG_MSG_VAL, //long  char value handl......

    HANDLE_HSENS_DEV_INFO_SERVICE = 0x40,
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, // characteristic handle
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,// char value handle

        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, // characteristic handl
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,// char value handle

        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, // characteristic handl
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,// char value handle

    HANDLE_HSENS_BATTERY_SERVICE = 0x60, // service handle
        HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, // characteristic handl
        HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL, // char value andle

    // Client Configuration
    HDLD_CURRENT_TIME_SERVICE_CURRENT_TIME_CLIENT_CONFIGURATION,
}hello_sensor_db_tags;


/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    BD_ADDR   remote_addr;              // remote peer device address
    uint32_t  timer_count;              // timer count
    uint32_t  fine_timer_count;         // fine timer count
    uint16_t  conn_id;                  // connection ID referenced by the stack
    uint16_t  peer_mtu;                 // peer MTU
    uint8_t   num_to_write;             // num msgs to send, incr on each button intr
    uint8_t   flag_indication_sent;     // indicates waiting for ack/cfm
    uint8_t   flag_stay_connected;      // stay connected or disconnect after all messages are sent
    uint8_t   battery_level;            // dummy battery level

} hello_sensor_state_t;


#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    BD_ADDR  bdaddr;                                /* BD address of the bonded host */
    uint16_t  characteristic_client_configuration;  /* Current value of the client configuration descriptor */
    uint8_t   number_of_blinks;                     /* Sensor config, number of times to blink the LEd when button is pushed. */
} host_info_t;
#pragma pack()

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;


#endif // _HELLO_SENSOR_H_

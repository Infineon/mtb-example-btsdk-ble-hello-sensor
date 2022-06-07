/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
* During initialization the app registers with LE stack to receive various
* notifications including bonding complete, connection status change and
* peer write.  When device is successfully bonded, application saves
* peer's Bluetooth Device address to the NVRAM.  Bonded device can also
* write in to client configuration descriptor of the notification
* characteristic.  That is also saved in the NVRAM.  When user pushes the
* button, notification/indication is sent to the bonded and registered host.
* User can also write to the characteristic configuration value. Number
* of LED blinks indicates the value written
*
*
* Features demonstrated
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - NVRAM read/write operation
*  - Sending data to the client
*  - Processing write requests from the client
 *
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Pair with client.
* 4. On the client side register for notifications
* 5. Push a button on the tag to send notifications to the client
* 6. Write the hello sensor characteristic configuration value from client
* 7. Number of LED blinks on hello sensor indicates value written by client
*/
#include <string.h>
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_bt_stack.h"
#include "wiced_transport.h"
#include "wiced_hal_puart.h"
#include "wiced_platform.h"

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif
#include "wiced_timer.h"
#include "hello_uuid.h"
#include "hello_sensor.h"


// #define ENABLE_HCI_TRACE 1 // configures HCI traces to be routed to the WICED HCI interface
/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HELLO_SENSOR_GATTS_MAX_CONN     1

#define APP_ADV_NAME wiced_bt_cfg_settings.device_name

#ifndef DEV_NAME
#define DEV_NAME "Hello"
#endif

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t hello_sensor_gatt_database[]=
{
    // Declare mandatory GATT service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    // Declare mandatory GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

    // Declare mandatory GAP service characteristic: Dev Name
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
            UUID_CHARACTERISTIC_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Declare mandatory GAP service characteristic: Appearance
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
            UUID_CHARACTERISTIC_APPEARANCE, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Declare proprietary Hello Service with 128 byte UUID
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_HELLO_SERVICE ),

    // Declare characteristic used to notify/indicate change
        CHARACTERISTIC_UUID128( HANDLE_HSENS_SERVICE_CHAR_NOTIFY, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
            UUID_HELLO_CHARACTERISTIC_NOTIFY, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE, GATTDB_PERM_READABLE ),

            // Declare client characteristic configuration descriptor
    // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection
            // for bonded devices.  Setting value to 1 tells this application to send notification
            // when value of the characteristic changes.  Value 2 is to allow indications.
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_AUTH_WRITABLE),

        // Declare characteristic Hello Configuration
    // The configuration consists of 1 bytes which indicates how many times to
    // blink the LED when user pushes the button.
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_BLINK, HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
            UUID_HELLO_CHARACTERISTIC_CONFIG, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE,
            GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ ),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVICE_DEVICE_INFORMATION ),

    // Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
            UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Handle 0x50: characteristic Model Number, handle 0x51 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
            UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Handle 0x52: characteristic System ID, handle 0x53 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
            UUID_CHARACTERISTIC_SYSTEM_ID, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE ),

    // Declare Battery service
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_BATTERY_SERVICE, 0x180F ),

    // Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value
        CHARACTERISTIC_UUID16( HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
            UUID_CHARACTERISTIC_BATTERY_LEVEL, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
};

size_t hello_sensor_gatt_database_size = sizeof(hello_sensor_gatt_database);

uint8_t hello_sensor_device_name[]          = DEV_NAME;                                          //GAP Service characteristic Device Name
uint8_t hello_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    hello_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0' };             //Notification Name
char    hello_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    hello_sensor_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t hello_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};

/* Holds the global state of the hello sensor application */
hello_sensor_state_t hello_sensor_state;

/* Holds the host info saved in the NVRAM */
host_info_t hello_sensor_hostinfo;
wiced_timer_t hello_sensor_second_timer;
wiced_timer_t hello_sensor_ms_timer;
wiced_timer_t hello_sensor_conn_idle_timer;

/* LED timer and counters */
wiced_timer_t hello_sensor_led_timer;
uint8_t       hello_sensor_led_blink_count  = 0;
uint16_t       hello_sensor_led_on_ms  = 0;
uint16_t       hello_sensor_led_off_ms  = 0;
#ifdef CYW20706A2
extern wiced_led_config_t platform_led[];
wiced_platform_led_t hello_sensor_led_pin = WICED_PLATFORM_LED_1;
#endif

#if defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20721B1) || defined(CYW20819A1) || defined(CYW55572A1)
extern wiced_platform_led_config_t platform_led[];
extern size_t led_count;
wiced_platform_led_number_t hello_sensor_led_pin = WICED_PLATFORM_LED_2;
#endif

#if defined(CYW20721B2) // 721B2
extern wiced_platform_led_config_t platform_led[];
extern size_t led_count;
#if defined (CYW920721B2EVK_03) || defined(CYW920721B2EVK_02)
wiced_platform_led_number_t hello_sensor_led_pin = WICED_PLATFORM_LED_1;
#else
wiced_platform_led_number_t hello_sensor_led_pin = WICED_PLATFORM_LED_2;
#endif
#endif

#if defined(CYW30739A0)
extern wiced_platform_led_config_t platform_led[];
extern size_t led_count;
wiced_platform_led_number_t hello_sensor_led_pin = WICED_PLATFORM_LED_1;
#endif

wiced_bt_gpio_numbers_t led_pin;

/* Attribute list of the hello sensor */
attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( hello_sensor_device_name ),         hello_sensor_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(hello_sensor_appearance_name),       hello_sensor_appearance_name },
    { HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,             sizeof(hello_sensor_char_notify_value),     hello_sensor_char_notify_value },
    { HANDLE_HSENS_SERVICE_CHAR_CFG_DESC,               2,                                          (void*)&hello_sensor_hostinfo.characteristic_client_configuration },
    { HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,              1,                                          &hello_sensor_hostinfo.number_of_blinks },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(hello_sensor_char_mfr_name_value),   hello_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(hello_sensor_char_model_num_value),  hello_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(hello_sensor_char_system_id_value),  hello_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                          &hello_sensor_state.battery_level },
};


static wiced_result_t           hello_sensor_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_bt_gatt_status_t          hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                     hello_sensor_set_advertisement_data(void);
void                            hello_sensor_send_message( void );
static void                     hello_sensor_gatts_increment_notify_value( void );
static void                     hello_sensor_timeout( TIMER_PARAM_TYPE count );
static void                     hello_sensor_fine_timeout( TIMER_PARAM_TYPE finecount );
static void                     hello_sensor_smp_bond_result( uint8_t result );
static void                     hello_sensor_encryption_changed( wiced_result_t result, uint8_t* bd_addr );
static void                     hello_sensor_interrupt_handler(void* user_data, uint8_t value );
static void                     hello_sensor_application_init( void );
static void                     hello_sensor_conn_idle_timeout ( TIMER_PARAM_TYPE arg );
#ifdef ENABLE_HCI_TRACE
static void                     hello_sensor_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif
static void                     hello_sensor_load_keys_for_address_resolution( void );
#ifndef CYW43012C0
static void                     hello_sensor_led_timeout( TIMER_PARAM_TYPE count );
void                            hello_sensor_led_blink(uint16_t on_ms, uint16_t off_ms, uint8_t num_of_blinks );
static void                     hello_sensor_interrput_config (void);
#endif


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#ifdef NO_PUART_SUPPORT
    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#endif

#ifdef ENABLE_HCI_TRACE
    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

#endif //WICED_BT_TRACE_ENABLE

    WICED_BT_TRACE( "Hello Sensor Start\n" );

    // Register call back and configuration with stack
#if BTSTACK_VER >= 0x03000001
    /* Create default heap */
    if (wiced_bt_create_heap("default_heap", NULL, BT_STACK_HEAP_SIZE, NULL, WICED_TRUE) == NULL)
    {
        WICED_BT_TRACE("create default heap error: size %d\n", BT_STACK_HEAP_SIZE);
        return;
    }
    wiced_bt_stack_init(hello_sensor_management_cback, &wiced_bt_cfg_settings);
#else
    wiced_bt_stack_init(hello_sensor_management_cback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
#endif
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void hello_sensor_application_init( void )
{

    wiced_result_t         result;

    WICED_BT_TRACE( "hello_sensor_application_init\n" );

#ifndef CYW43012C0
    hello_sensor_interrput_config();
#endif

    // init gatt
    hello_sensor_gatt_init();


#ifdef ENABLE_HCI_TRACE
    wiced_bt_dev_register_hci_trace( hello_sensor_hci_trace_cback );
#endif
    /* Starting the app timers , seconds timer and the ms timer  */
    wiced_init_timer(&hello_sensor_second_timer, hello_sensor_timeout, 0, WICED_SECONDS_PERIODIC_TIMER);
    wiced_start_timer( &hello_sensor_second_timer, HELLO_SENSOR_APP_TIMEOUT_IN_SECONDS );
    wiced_init_timer(&hello_sensor_ms_timer, hello_sensor_fine_timeout, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);

    // set fine timeout to 25ms, to accomodate 12.5ms min resolution on 20706A2
    wiced_start_timer( &hello_sensor_ms_timer, 25 * HELLO_SENSOR_APP_FINE_TIMEOUT_IN_MS );

    /* init the idle timer (but do not start yet) */
    wiced_init_timer(&hello_sensor_conn_idle_timer, hello_sensor_conn_idle_timeout, 0, WICED_SECONDS_TIMER);

    /* Load previous paired keys for address resolution */
    hello_sensor_load_keys_for_address_resolution();

#if defined(CYW20706A2) || defined(CYW20735B0)
    /* Enable privacy to advertise with RPA */
    wiced_bt_ble_enable_privacy ( WICED_TRUE );
#endif
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    hello_sensor_set_advertisement_data();

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );

    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    hello_sensor_state.flag_stay_connected = 1;
#ifndef CYW43012C0
#ifdef CYW20706A2
    platform_led_init();
#endif
    wiced_init_timer(&hello_sensor_led_timer, hello_sensor_led_timeout, 0, WICED_MILLI_SECONDS_TIMER);
    led_pin = HELLO_SENSOR_LED_GPIO;
#endif
}

#ifndef CYW43012C0
static void hello_sensor_interrput_config (void)
{
#if defined(CYW20706A2) || defined(CYW20719B0)
    wiced_bt_app_init();
#endif

    /* Configure buttons available on the platform (pin should be configured before registering interrupt handler ) */
#if defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW20706A2)
    wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON, HELLO_SENSOR_GPIO_BUTTON_SETTINGS, HELLO_SENSOR_GPIO_BUTTON_PRESSED_VALUE );
    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON, hello_sensor_interrupt_handler, NULL );
#else
    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, hello_sensor_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);
#endif
}
#endif

/*
 *  Pass protocol traces up through the UART
 */
#ifdef ENABLE_HCI_TRACE
void hello_sensor_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
 #if BTSTACK_VER >= 0x03000001
    wiced_transport_send_hci_trace( type, p_data, length );
 #else
    wiced_transport_send_hci_trace( NULL, type, length, p_data );
 #endif
}
#endif

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void hello_sensor_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t hello_service_uuid[LEN_UUID_128] = { UUID_HELLO_SERVICE };

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = hello_service_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (const char *) APP_ADV_NAME);
    adv_elem[num_elem].p_data       = (uint8_t *)APP_ADV_NAME;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * This function is invoked when advertisements stop.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 */
void hello_sensor_advertisement_stopped( void )
{
    wiced_result_t result;

    if ( hello_sensor_state.flag_stay_connected && !hello_sensor_state.conn_id )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", result );
    }
    else
    {
        WICED_BT_TRACE( "ADV stop\n");
    }

    UNUSED_VARIABLE(result);
}

#ifndef CYW43012C0
/*
 * The function invoked on timeout of led timer.
 */
void hello_sensor_led_timeout( TIMER_PARAM_TYPE count )
{
    static wiced_bool_t led_on = WICED_FALSE;
    if ( led_on )
    {
        wiced_hal_gpio_set_pin_output(led_pin, GPIO_PIN_OUTPUT_HIGH);
        if (--hello_sensor_led_blink_count)
        {
            led_on = WICED_FALSE;
            wiced_start_timer( &hello_sensor_led_timer, hello_sensor_led_off_ms );
        }
    }
    else
    {
        led_on = WICED_TRUE;
        wiced_hal_gpio_set_pin_output(led_pin, GPIO_PIN_OUTPUT_LOW);
        wiced_start_timer( &hello_sensor_led_timer, hello_sensor_led_on_ms );
    }
}

/**
 * Blinks the LED
 */
void hello_sensor_led_blink(uint16_t on_ms, uint16_t off_ms, uint8_t num_of_blinks )
{
    if (num_of_blinks
#if defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20719B1) || defined(CYW20719B2) || defined(CYW20721B1) || defined(CYW20721B2) || defined(CYW20819A1) || defined(CYW30739A0) || defined(CYW55572A1)
            && hello_sensor_led_pin < led_count
#endif
        )
    {
        hello_sensor_led_blink_count = num_of_blinks;
        hello_sensor_led_off_ms = off_ms;
        hello_sensor_led_on_ms = on_ms;
        wiced_hal_gpio_set_pin_output(led_pin, GPIO_PIN_OUTPUT_LOW);
        wiced_stop_timer(&hello_sensor_led_timer);
        wiced_start_timer(&hello_sensor_led_timer,on_ms);
    }
}
#endif

/*
 * The function invoked on timeout of app seconds timer.
 */
void hello_sensor_timeout( TIMER_PARAM_TYPE count )
{
    hello_sensor_state.timer_count++;

    // print for first 10 seconds, then once every 10 seconds thereafter
    if ((hello_sensor_state.timer_count <= 10) || (hello_sensor_state.timer_count % 10 == 0))
        WICED_BT_TRACE("hello_sensor_timeout: %d, ft:%d \n",
                hello_sensor_state.timer_count, hello_sensor_state.fine_timer_count );
}

/*
 * The function invoked on timeout of app milliseconds fine timer
 */
void hello_sensor_fine_timeout( TIMER_PARAM_TYPE finecount )
{
    hello_sensor_state.fine_timer_count++;
}

/*
 * Process SMP bonding result. If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */

void hello_sensor_smp_bond_result( uint8_t result )
{
    wiced_result_t status;
    uint8_t written_byte = 0;
    WICED_BT_TRACE( "hello_sensor, bond result: %d\n", result );

    /* Bonding success */
    if ( result == WICED_BT_SUCCESS )
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy( hello_sensor_hostinfo.bdaddr, hello_sensor_state.remote_addr, sizeof( BD_ADDR ) );

        /* Write to NVRAM */
        written_byte = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &status );
        WICED_BT_TRACE("NVRAM write: %d\n", written_byte);
    }

    UNUSED_VARIABLE(written_byte);
}


/*
 * Process notification from stack that encryption has been set. If connected
 * client is registered for notification or indication, it is a good time to
 * send it out
 */
void hello_sensor_encryption_changed( wiced_result_t result, uint8_t* bd_addr )
{
    WICED_BT_TRACE( "encryp change bd ( %B ) res: %d ", hello_sensor_hostinfo.bdaddr,  result);

    /* Connection has been encrypted meaning that we have correct/paired device
     * restore values in the database
     */
    wiced_hal_read_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &result );

    // If there are outstanding messages that we could not send out because
    // connection was not up and/or encrypted, send them now.  If we are sending
    // indications, we can send only one and need to wait for ack.
    while ( ( hello_sensor_state.num_to_write != 0 ) && !hello_sensor_state.flag_indication_sent )
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }

    // If configured to disconnect after delivering data, start idle timeout
    // to do disconnection
    if ( ( !hello_sensor_state.flag_stay_connected ) && !hello_sensor_state.flag_indication_sent  )
    {
        if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_conn_idle_timer);
            wiced_start_timer(&hello_sensor_conn_idle_timer, HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
        }
    }
}

#ifndef CYW43012C0
/*
 * Three Interrupt inputs (Buttons) can be handled here.
 * If the following value == 1, Button is pressed. Different than initial value.
 * If the following value == 0, Button is depressed. Same as initial value.
 * Button1 : ( value & 0x01 )
 * Button2 : ( value & 0x02 ) >> 1
 * Button3 : ( value & 0x04 ) >> 2
 */
void hello_sensor_interrupt_handler(void* user_data, uint8_t value )
{
    static uint32_t previous_button_pressed_timer = 0;
    static uint32_t current_button_pressed_timer = 0;

    current_button_pressed_timer = hello_sensor_state.fine_timer_count;

    // 50ms gap(2x25) to rid of any button signal bounce issues
    if ((current_button_pressed_timer - previous_button_pressed_timer) > 2)
    {
        WICED_BT_TRACE("button pressed\n");
        previous_button_pressed_timer = current_button_pressed_timer;

        // Blink as configured
        hello_sensor_led_blink( 250, 250, hello_sensor_hostinfo.number_of_blinks);

        /* Increment the last byte of the hello sensor notify value */
        hello_sensor_gatts_increment_notify_value();

        /* Remember how many messages we need to send */
        hello_sensor_state.num_to_write++;

        /* If connection is down, start high duty advertisements, so client can connect */
        if ( hello_sensor_state.conn_id == 0 )
        {
            wiced_result_t result;

            WICED_BT_TRACE( "ADV start high\n");

            result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

            WICED_BT_TRACE( "wiced_bt_start_advertisements:%d\n", result );
            UNUSED_VARIABLE(result);
            return;
        }

        /*
         * Connection up.
         * Send message if client registered to receive indication
         * or notification. After we send an indication wait for the ack
         * before we can send anything else
         */
        while ( ( hello_sensor_state.num_to_write != 0 ) && !hello_sensor_state.flag_indication_sent )
        {
            hello_sensor_state.num_to_write--;
            hello_sensor_send_message();
        }

        // if we sent all messages, start connection idle timer to disconnect
        if ( !hello_sensor_state.flag_stay_connected && !hello_sensor_state.flag_indication_sent )
        {
            if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer) )
            {
                wiced_stop_timer(&hello_sensor_conn_idle_timer);
                wiced_start_timer(&hello_sensor_conn_idle_timer, HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
            }
        }
    }
}
#endif

/*
 * Function on connection idle timeout initiates the gatt disconnect
 */
void hello_sensor_conn_idle_timeout ( TIMER_PARAM_TYPE arg )
{
    WICED_BT_TRACE( "hello_sensor_conn_idle_timeout\n" );

    /* Stopping the app timers */
    wiced_stop_timer(&hello_sensor_second_timer);
    wiced_stop_timer(&hello_sensor_ms_timer);

    /* Initiating the gatt disconnect */
    wiced_bt_gatt_disconnect( hello_sensor_state.conn_id );
}

/*
 * hello_sensor bt/ble link management callback
 */
wiced_result_t hello_sensor_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_encryption_status_t    *p_status;
    wiced_bt_dev_ble_pairing_info_t     *p_info;
    wiced_bt_ble_advert_mode_t          *p_mode;
    wiced_bt_device_address_t           id_addr;
    uint8_t                             *p_keys;
    wiced_result_t                      result = WICED_BT_SUCCESS;

    WICED_BT_TRACE("hello_sensor_management_cback: %x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hello_sensor_application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("Numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap      = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data          = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req          = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size      = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys         = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            WICED_BT_TRACE( "Pairing Complete: %d ", p_info->reason);
            hello_sensor_smp_bond_result( p_info->reason );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
            wiced_hal_write_nvram ( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("keys save to NVRAM %B result: %d \n", p_keys, result);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            {
                wiced_bt_device_link_keys_t link_keys;
                p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
                wiced_hal_read_nvram( HELLO_SENSOR_PAIRED_KEYS_VS_ID,
                                      sizeof(wiced_bt_device_link_keys_t),
                                      (uint8_t *)&link_keys,
                                      &result );

                WICED_BT_TRACE("keys read from NVRAM %B req BDA %B result: %d \n", link_keys.bd_addr, p_event_data->paired_device_link_keys_request.bd_addr, result);

                // Break if link key retrival is failed or link key is not available.
                if (result != WICED_BT_SUCCESS)
                    break;

                // Compare the BDA
                if( memcmp(&(link_keys.bd_addr), &(p_event_data->paired_device_link_keys_request.bd_addr), sizeof(wiced_bt_device_address_t) ) == 0 )
                {
                    memcpy(p_keys, (uint8_t *)&link_keys, sizeof(wiced_bt_device_link_keys_t));
                }
                else
                {
                    result = WICED_BT_ERROR;
                }
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram ( HELLO_SENSOR_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( HELLO_SENSOR_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
            WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_status = &p_event_data->encryption_status;
            WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d", p_status->bd_addr, p_status->result);
            hello_sensor_encryption_changed( p_status->result, p_status->bd_addr );
        break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
            if ( *p_mode == BTM_BLE_ADVERT_OFF )
            {
                hello_sensor_advertisement_stopped();
            }
            break;

    default:
            break;
    }

    return result;
}


/*
 * Check if client has registered for notification/indication
 * and send message if appropriate
 */
void hello_sensor_send_message( void )
{
    WICED_BT_TRACE( "hello_sensor_send_message: CCC:%d\n", hello_sensor_hostinfo.characteristic_client_configuration );

    /* If client has not registered for indication or notification, no action */
    if ( hello_sensor_hostinfo.characteristic_client_configuration == 0 )
    {
        return;
    }
    else if ( hello_sensor_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
    {
        uint8_t *p_attr = (uint8_t*)hello_sensor_char_notify_value;

        wiced_bt_gatt_send_notification( hello_sensor_state.conn_id, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, sizeof(hello_sensor_char_notify_value), p_attr );
    }
    else
    {
        if ( !hello_sensor_state.flag_indication_sent )
        {
            uint8_t *p_attr = (uint8_t *)hello_sensor_char_notify_value;

            hello_sensor_state.flag_indication_sent = TRUE;

            wiced_bt_gatt_send_indication( hello_sensor_state.conn_id, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL, sizeof(hello_sensor_char_notify_value), p_attr );
        }
    }
}

/*
 * Find attribute description by handle
 */
attribute_t * hello_sensor_get_attribute( uint16_t handle )
{
    uint32_t i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}

/* This function is invoked when connection is established */
wiced_bt_gatt_status_t hello_sensor_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "hello_sensor_conn_up %B id:%d\n:", p_status->bd_addr, p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    hello_sensor_state.conn_id = p_status->conn_id;
    memcpy(hello_sensor_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop idle timer */
    wiced_stop_timer(&hello_sensor_conn_idle_timer);

    /* Saving host info in NVRAM  */
    memcpy( hello_sensor_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );
    hello_sensor_hostinfo.characteristic_client_configuration = 0;
    hello_sensor_hostinfo.number_of_blinks                    = 0;

    {
        uint8_t bytes_written = 0;
        bytes_written = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &result );

        WICED_BT_TRACE("NVRAM write %d\n", bytes_written);
        UNUSED_VARIABLE(bytes_written);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
wiced_bt_gatt_status_t hello_sensor_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "connection_down %B conn_id:%d reason:%d\n", hello_sensor_state.remote_addr, p_status->conn_id, p_status->reason );

    /* Resetting the device info */
    memset( hello_sensor_state.remote_addr, 0, 6 );
    hello_sensor_state.conn_id = 0;

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if ( hello_sensor_state.flag_stay_connected )
    {
        result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );
    }

    UNUSED_VARIABLE(result);

    return WICED_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
wiced_bt_gatt_status_t hello_sensor_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return hello_sensor_gatts_connection_up( p_status );
    }

    return hello_sensor_gatts_connection_down( p_status );
}


/*
 * Keep number of the button pushes in the last byte of the Hello message.
 * That will guarantee that if client reads it, it will have correct data.
 */
void hello_sensor_gatts_increment_notify_value( void )
{
    /* Getting the last byte */
    int last_byte = sizeof( hello_sensor_char_notify_value ) - 1 ;
    char c = hello_sensor_char_notify_value[last_byte];

    c++;
    if ( (c < '0') || (c > '9') )
    {
        c = '0';
    }
    hello_sensor_char_notify_value[last_byte] = c;
}

static void hello_sensor_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result;
    uint8_t                     *p;

    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram( HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if(result == WICED_BT_SUCCESS)
    {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys, link_keys.key_data.ble_addr_type );
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );
#endif
    }
    WICED_BT_TRACE("hello_sensor_load_keys_for_address_resolution %B result:%d \n", p, result );
}

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

/* This file is applicable for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0 */

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
#include "wiced_timer.h"
#include "wiced_platform.h"

#define WICED_GPIO_PIN_OUTPUT_HIGH GPIO_PIN_OUTPUT_HIGH
#define WICED_GPIO_PIN_OUTPUT_LOW  GPIO_PIN_OUTPUT_LOW
#include "wiced_gki.h"
#if !defined(CYW20835B1) && !defined(CYW20719B1) && !defined(CYW20721B1) && !defined(CYW20819A1) && !defined(CYW20719B2) && !defined(CYW20721B2) && !defined(CYW30739A0)
#include "wiced_bt_app_common.h"
#endif

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif
#include "wiced_timer.h"
#include "hello_sensor.h"

/* Extern Functions */
#ifndef CYW43012C0
void                     hello_sensor_led_blink(uint16_t on_ms, uint16_t off_ms, uint8_t num_of_blinks );
#endif
attribute_t * hello_sensor_get_attribute( uint16_t handle );
void hello_sensor_send_message( void );
wiced_bt_gatt_status_t hello_sensor_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status );
wiced_bt_gatt_status_t hello_sensor_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_req );
wiced_bt_gatt_status_t hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t hello_sensor_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data );

/* Extern variables */
extern hello_sensor_state_t hello_sensor_state;
extern host_info_t hello_sensor_hostinfo;
extern wiced_timer_t hello_sensor_conn_idle_timer;

/*
 * helper function to init GATT
 *
 */
void hello_sensor_gatt_init()
{
    wiced_bt_gatt_status_t gatt_status;
     /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(hello_sensor_gatts_callback);
    WICED_BT_TRACE( "wiced_bt_gatt_register: %d\n", gatt_status );
    /*  Tell stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init( hello_sensor_gatt_database, hello_sensor_gatt_database_size );
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are ommitted.
 */
wiced_bt_gatt_status_t hello_sensor_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hello_sensor_gatts_conn_status_cb( &p_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hello_sensor_gatts_req_cb( &p_data->attribute_request );
        break;

    case GATT_OPERATION_CPLT_EVT:
        result = hello_sensor_gatt_op_comp_cb( &p_data->operation_complete );
        break;

    default:
        break;
    }
    return result;
}


/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = hello_sensor_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( hello_sensor_state.battery_level++ > 5)
        {
            hello_sensor_state.battery_level = 0;
        }
    }


    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    uint8_t                nv_update = WICED_FALSE;

    WICED_BT_TRACE("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len );

    switch ( p_data->handle )
    {
    /* By writing into Characteristic Client Configuration descriptor
     * peer can enable or disable notification or indication */
    case HANDLE_HSENS_SERVICE_CHAR_CFG_DESC:
        if ( p_data->val_len != 2 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        hello_sensor_hostinfo.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );
        nv_update = WICED_TRUE;
        break;

    case HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL:
        if ( p_data->val_len != 1 )
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        hello_sensor_hostinfo.number_of_blinks = p_attr[0];
        if ( hello_sensor_hostinfo.number_of_blinks != 0 )
        {
            WICED_BT_TRACE( "hello_sensor_write_handler:num blinks: %d\n", hello_sensor_hostinfo.number_of_blinks );
#ifndef CYW43012C0
            hello_sensor_led_blink (250,250,hello_sensor_hostinfo.number_of_blinks);
#endif

            nv_update = WICED_TRUE;
        }
        break;

    default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    if ( nv_update )
    {
        wiced_result_t rc;
        int bytes_written = wiced_hal_write_nvram( HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo), (uint8_t*)&hello_sensor_hostinfo, &rc );
        WICED_BT_TRACE("NVRAM write:%d rc:%d", bytes_written, rc);
    }

    return result;
}

/*
 * Write Execute Procedure
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_exec_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg )
{
    WICED_BT_TRACE("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_mtu_handler( uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm. If client wanted us to use indication instead of
 * notifications we have to wait for confirmation after every message sent.
 * For example if user pushed button twice very fast
 * we will send first message, then
 * wait for confirmation, then
 * send second message, then
 * wait for confirmation and
 * if configured start idle timer only after that.
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_conf_handler( uint16_t conn_id, uint16_t handle )
{
    WICED_BT_TRACE( "hello_sensor_indication_cfm, conn %d hdl %d\n", conn_id, handle );

    if ( !hello_sensor_state.flag_indication_sent )
    {
        WICED_BT_TRACE("Hello: Wrong Confirmation!");
        return WICED_BT_GATT_SUCCESS;
    }

    hello_sensor_state.flag_indication_sent = 0;

    /* We might need to send more indications */
    if ( hello_sensor_state.num_to_write )
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }
    /* if we sent all messages, start connection idle timer to disconnect */
    if ( !hello_sensor_state.flag_stay_connected && !hello_sensor_state.flag_indication_sent )
    {
        if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer) )
        {
            wiced_stop_timer(&hello_sensor_conn_idle_timer);
            wiced_start_timer(&hello_sensor_conn_idle_timer, HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
        }
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT request from the peer
 */
wiced_bt_gatt_status_t hello_sensor_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE( "hello_sensor_gatts_req_cb. conn %d, type %d\n", p_req->conn_id, p_req->request_type );

    switch ( p_req->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        result = hello_sensor_gatts_req_read_handler( p_req->conn_id, &(p_req->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = hello_sensor_gatts_req_write_handler( p_req->conn_id, &(p_req->data.write_req) );
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = hello_sensor_gatts_req_write_exec_handler( p_req->conn_id, p_req->data.exec_write );
        break;

    case GATTS_REQ_TYPE_MTU:
        result = hello_sensor_gatts_req_mtu_handler( p_req->conn_id, p_req->data.mtu );
        break;

    case GATTS_REQ_TYPE_CONF:
        result = hello_sensor_gatts_req_conf_handler( p_req->conn_id, p_req->data.handle );
        break;

   default:
        break;
    }


    return result;
}

/*
 * GATT operation started by the sensor has been completed
 */
wiced_bt_gatt_status_t hello_sensor_gatt_op_comp_cb( wiced_bt_gatt_operation_complete_t *p_data )
{
    wiced_result_t              status;

    WICED_BT_TRACE("==>hello_sensor conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    switch ( p_data->op )
    {
        case GATTC_OPTYPE_NOTIFICATION:
            p_data->response_data.att_value.p_data[p_data->response_data.att_value.len] = 0;
            WICED_BT_TRACE( "Get Notification from central conn_id %d, data[%d] = %s:\n", p_data->conn_id, p_data->response_data.att_value.len, p_data->response_data.att_value.p_data);
            break;

        default:
            WICED_BT_TRACE( "default:\n");
            break;
    }

    UNUSED_VARIABLE(status);
    return WICED_BT_GATT_SUCCESS;
}

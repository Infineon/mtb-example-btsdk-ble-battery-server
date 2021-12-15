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

/* This file is gatts functions for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0 */

#include "wiced_bt_gatt.h"
#include "wiced_hal_nvram.h"
#include "battery_server.h"
#include "wiced_bt_trace.h"

/*
 * Process Read request or command from peer device
 */
static wiced_bt_gatt_status_t battery_server_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    gatt_db_lookup_table_t *p_attribute;
    int                         to_copy;

    if ( ( p_attribute = battery_server_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    to_copy = *p_read_data->p_val_len;

//    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d dlen=%d\n", conn_id, p_read_data->handle, p_read_data->offset, to_copy, p_attribute->cur_len);

    // is offset valid?
    if ( p_read_data->offset > p_attribute->cur_len )
    {
        return WICED_BT_GATT_INVALID_OFFSET;
    }
    else if (to_copy + p_read_data->offset > p_attribute->cur_len)
    {
        to_copy = p_attribute->cur_len - p_read_data->offset;
    }

    if (to_copy > 0 )
    {
        uint8_t *from = ((uint8_t *)p_attribute->p_data) + p_read_data->offset;
        memcpy( p_read_data->p_val, from, to_copy);
    }
    *p_read_data->p_val_len = to_copy;

    return WICED_BT_GATT_SUCCESS;
}

/* GATT event handler */
wiced_bt_gatt_status_t battery_server_gatt_cback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = battery_server_gatts_conn_status_cb( &p_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = battery_server_gatts_req_cb( &p_data->attribute_request );
            break;

        default:
            break;
    }
    return result;
}

wiced_bt_gatt_status_t battery_server_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

//    WICED_BT_TRACE( "battery_server_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type );

    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        result = battery_server_gatts_req_read_handler( p_data->conn_id, &(p_data->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = battery_server_gatts_write_handle( p_data->data.write_req.handle, p_data->data.write_req.offset, p_data->data.write_req.val_len, p_data->data.write_req.p_val );
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        break;

    case GATTS_REQ_TYPE_MTU:
        break;

    case GATTS_REQ_TYPE_CONF:
        WICED_BT_TRACE("Confirm received\n");
        battery_server_state.indication_sent = WICED_FALSE;
        result = WICED_BT_GATT_SUCCESS;
        break;

   default:
        break;
    }

    return result;
}

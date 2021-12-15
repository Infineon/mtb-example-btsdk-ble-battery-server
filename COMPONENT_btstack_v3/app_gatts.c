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

/* This file is applicable for all devices with BTSTACK version 3.0 and greater, for example 55572 */

#include "wiced_bt_gatt.h"
#include "wiced_memory.h"
#include "bt_types.h"
#include "wiced_bt_trace.h"
#include "battery_server.h"

/*
 * Process Read request or read blob from peer device
 */
static wiced_bt_gatt_status_t battery_server_gatts_req_read_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *p_attribute;
    uint8_t *from;

    if ( ( p_attribute = battery_server_get_attribute ( p_read_req->handle) ) == NULL )
    {
        WICED_BT_TRACE("[%s]  attr not found handle: 0x%04x\n", __FUNCTION__, p_read_req->handle);
        wiced_bt_gatt_server_send_error_rsp ( conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_HANDLE );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

//    WICED_BT_TRACE( "[%s] conn_id: %d handle:0x%04x offset:%d len:%d\n", __FUNCTION__, conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy );

    // is offset valid?
    if ( p_read_req->offset > p_attribute->cur_len )
    {
        WICED_BT_TRACE ("[%s] offset:%d larger than attribute length:%d\n", __FUNCTION__, p_read_req->offset, p_attribute->cur_len);

        wiced_bt_gatt_server_send_error_rsp (conn_id, opcode, p_read_req->handle, WICED_BT_GATT_INVALID_OFFSET);
        return (WICED_BT_GATT_INVALID_OFFSET);
    }
    else if (len_requested + p_read_req->offset > p_attribute->cur_len)
    {
        len_requested = p_attribute->cur_len - p_read_req->offset;
    }

    from = ((uint8_t *)p_attribute->p_data) + p_read_req->offset;

    wiced_bt_gatt_server_send_read_handle_rsp (conn_id, opcode, len_requested, from, NULL);

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

        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_data->buffer_request.buffer.p_app_rsp_buffer = battery_server_alloc_buffer (p_data->buffer_request.len_requested);
            p_data->buffer_request.buffer.p_app_ctxt       = battery_server_free_buffer;
            result = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
                pfn_free(p_data->buffer_xmitted.p_app_data);

            result = WICED_BT_GATT_SUCCESS;
        }
        break;

        default:
            break;
    }
    return result;
}

/*
 * Process write read-by-type request from peer device
 */
wiced_bt_gatt_status_t battery_server_gatts_req_read_by_type_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
    wiced_bt_gatt_read_by_type_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *p_attribute;
    uint16_t    attr_handle = p_read_req->s_handle;
    uint8_t     *p_rsp = battery_server_alloc_buffer ( len_requested );
    uint8_t     pair_len = 0;
    int         used = 0;

    if ( p_rsp == NULL )
    {
        WICED_BT_TRACE ( "[%s]  no memory len_requested: %d!!\n", __FUNCTION__, len_requested );

        wiced_bt_gatt_server_send_error_rsp ( conn_id, opcode, attr_handle, WICED_BT_GATT_INSUF_RESOURCE );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type ( attr_handle, p_read_req->e_handle, &p_read_req->uuid );

        if (attr_handle == 0)
            break;

        if ( ( p_attribute = battery_server_get_attribute ( attr_handle ) ) == NULL )
        {
            WICED_BT_TRACE ( "[%s]  found type but no attribute ??\n", __FUNCTION__ );
            wiced_bt_gatt_server_send_error_rsp ( conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_ERR_UNLIKELY );
            battery_server_free_buffer( p_rsp );
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream( p_rsp + used, len_requested - used, &pair_len,
                                                            attr_handle, p_attribute->cur_len, p_attribute->p_data );
            if ( filled == 0 )
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if ( used == 0 )
    {
        WICED_BT_TRACE ( "[%s]  attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\n",
                        __FUNCTION__, p_read_req->s_handle, p_read_req->e_handle, p_read_req->uuid.uu.uuid16 );

        wiced_bt_gatt_server_send_error_rsp ( conn_id, opcode, p_read_req->s_handle, WICED_BT_GATT_INVALID_HANDLE );
        battery_server_free_buffer ( p_rsp );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp ( conn_id, opcode, pair_len, used, p_rsp, battery_server_free_buffer );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write read multi request from peer device
 */
wiced_bt_gatt_status_t battery_server_gatts_req_read_multi_handler (uint16_t conn_id, wiced_bt_gatt_opcode_t opcode,
        wiced_bt_gatt_read_multiple_req_t *p_read_req, uint16_t len_requested)
{
    gatt_db_lookup_table_t *p_attribute;
    uint8_t     *p_rsp = battery_server_alloc_buffer( len_requested );
    int         used = 0;
    int         xx;
    uint16_t    handle;
    int         filled;

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE ( "[%s]  no memory len_requested: %d!!\n", __FUNCTION__, len_requested );

        wiced_bt_gatt_server_send_error_rsp ( conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INSUF_RESOURCE );

        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream( p_read_req->p_handle_stream, xx );
        if ( ( p_attribute = battery_server_get_attribute ( handle ) ) == NULL )
        {
            WICED_BT_TRACE ("[%s]  no handle 0x%04xn", __FUNCTION__, handle);
            wiced_bt_gatt_server_send_error_rsp ( conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE );
            battery_server_free_buffer( p_rsp );
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        filled = wiced_bt_gatt_put_read_multi_rsp_in_stream( opcode, p_rsp + used, len_requested - used,
                                                            p_attribute->handle, p_attribute->cur_len, p_attribute->p_data );

        if ( !filled )
        {
            break;
        }
        used += filled;
    }

    if (used == 0)
    {
        WICED_BT_TRACE ( "[%s] no attr found\n", __FUNCTION__ );

        wiced_bt_gatt_server_send_error_rsp ( conn_id, opcode, *p_read_req->p_handle_stream, WICED_BT_GATT_INVALID_HANDLE );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp ( conn_id, opcode, used, p_rsp, battery_server_free_buffer );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Write Execute Procedure
 */
wiced_bt_gatt_status_t battery_server_gatts_req_write_exec_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg )
{
    WICED_BT_TRACE( "write exec: flag:%d\n", exec_falg );
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t battery_server_gatts_req_mtu_handler( uint16_t conn_id, uint16_t mtu )
{
    WICED_BT_TRACE( "req_mtu: %d\n", mtu );
    wiced_bt_gatt_server_send_mtu_rsp( conn_id, mtu, wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size );
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT request from the peer
 */
wiced_bt_gatt_status_t battery_server_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

//    WICED_BT_TRACE( "battery_server_gatts_req_cb. conn %d, opcode %d\n", p_data->conn_id, p_data->opcode );

    switch ( p_data->opcode )
    {
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        result = battery_server_gatts_req_read_handler( p_data->conn_id, p_data->opcode, &p_data->data.read_req, p_data->len_requested );
        break;

    case GATT_REQ_READ_BY_TYPE:
        result = battery_server_gatts_req_read_by_type_handler ( p_data->conn_id, p_data->opcode, &p_data->data.read_by_type, p_data->len_requested );
        break;

    case GATT_REQ_READ_MULTI:
    case GATT_REQ_READ_MULTI_VAR_LENGTH:
        result = battery_server_gatts_req_read_multi_handler ( p_data->conn_id, p_data->opcode, &p_data->data.read_multiple_req, p_data->len_requested );
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
        result = battery_server_gatts_write_handle( p_data->data.write_req.handle, p_data->data.write_req.offset, p_data->data.write_req.val_len, p_data->data.write_req.p_val );
        if ( ( p_data->opcode == GATT_REQ_WRITE ) &&  ( result == WICED_BT_GATT_SUCCESS ) )
        {
            wiced_bt_gatt_write_req_t   *p_write_request = &p_data->data.write_req;
            wiced_bt_gatt_server_send_write_rsp( p_data->conn_id, p_data->opcode, p_write_request->handle );
        }
        break;

    case GATT_REQ_MTU:
        result = battery_server_gatts_req_mtu_handler( p_data->conn_id, p_data->data.remote_mtu );
        break;

    case GATT_HANDLE_VALUE_CONF:
        WICED_BT_TRACE("Confirm received\n");
        battery_server_state.indication_sent = WICED_FALSE;
        result = WICED_BT_GATT_SUCCESS;
        break;

   default:
        break;
    }

    return result;
}

uint8_t *battery_server_alloc_buffer(uint16_t len)
{
    uint8_t *p = (uint8_t *) wiced_bt_get_buffer( len );
    WICED_BT_TRACE( "[%s] len %d alloc 0x%x", __FUNCTION__, len, p );

    return p;
}

void battery_server_free_buffer(uint8_t *p_data)
{
    wiced_bt_free_buffer( p_data );

    WICED_BT_TRACE( "[%s] 0x%x", __FUNCTION__, p_data );
}

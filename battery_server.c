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
 * BLE Battery Service Server Sample Application
 *
 * Features demonstrated
 *  -Battery Service implementation. For details refer to BT SIG Battery Service Profile 1.0 spec.
 *
 * On startup this demo:
 *  - Initializes the Battery Service GATT database
 *  - Begins advertising
 *  - Waits for GATT clients to connect
 *
 * To test the app, work through the following steps.
 * 1. Plug the WICED eval board into your computer
 * 2. Build and download the application (to the WICED board)
 * 3. On application start the device acts as a GATT server and advertises itself as Battery Service.
 * 4. Connect to Battery Service using one of the LE clients (LEExplorer(android)) or (BLE Utility(Apple Store))
 *    or battery_server_client application.
 * 5. Once connected the client can read Battery levels.
 * 6. If the client enables the notification, the Battery Server will send battery level every 2 secs.
 * 7. Battery Level starts from 100 and keeps decrementing to 0. The Battery Server App is designed such that,
 *    once it hits 0, it will be rolled back to 100.
 */

#include "sparcommon.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_puart.h"
#include "battery_server.h"

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
/* Holds global state of the App */
battery_server_state_t battery_server_state = {
#ifdef WICED_BT_TRACE_ENABLE
    {"Level",
 #ifdef BAS_1_1
     "Level Status",
     "Estimated Service Date",
     "Critical Status",
     "Energy Status",
     "Time Status",
     "Health Status",
     "Health Information",
     "Information",
     "Manufacture Name",
     "Manufacture Number",
     "Serial Number",
 #endif
    },
#endif
    {{HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG, MAX_BATTERY_LEVEL}, // Mondatary for v1.0
#ifdef BAS_1_1
     {HDLC_BAS_BATTERY_LEVEL_STATUS_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_ESTIMATED_SERVICE_DATE_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_BATTERY_CRITICAL_STATUS_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_BATTERY_ENERGY_STATUS_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_BATTERY_TIME_STATUS_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_BATTERY_HEALTH_STATUS_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_BATTERY_HEALTH_INFO_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_BATTERY_INFO_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_MANUFACTURE_NAME_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_MANUFACTURE_NUMBER_CLIENT_CHAR_CONFIG},
     {HDLC_BAS_SERIAL_NUMBER_CLIENT_CHAR_CONFIG},
#endif
     }};

/* Holds the host info saved in the NVRAM */
host_info_t battery_server_hostinfo;

wiced_timer_t battery_server_timer;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t battery_server_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;
    uint8_t        bytes_written = 0;

    WICED_BT_TRACE( "battery_server_conn_up %B id:%d\n:", p_status->bd_addr, p_status->conn_id);

    /* clear indication_sent flag */
    battery_server_state.indication_sent = WICED_FALSE;

    /* Update the connection handler.  Save address of the connected device. */
    battery_server_state.conn_id = p_status->conn_id;
    memcpy(battery_server_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );

    WICED_BT_TRACE( "Stopping Advertisements%d\n", result );

    /* Updating the bd address in the  host info in NVRAM  */
    memcpy( battery_server_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t battery_server_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "connection_down %B conn_id:%d reason:%d\n", battery_server_state.remote_addr, p_status->conn_id, p_status->reason );

    /* Resetting the device info */
    memset( battery_server_state.remote_addr, 0, 6 );
    battery_server_state.conn_id = 0;
    wiced_stop_timer( &battery_server_timer );

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );

    return WICED_BT_SUCCESS;
}

wiced_bt_gatt_status_t battery_server_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return battery_server_gatts_connection_up( p_status );
    }

    return battery_server_gatts_connection_down( p_status );
}

/*
 *  Pass protocol traces up through the UART
 */
#ifdef ENABLE_HCI_TRACE
static void battery_server_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
    app_transport_send_hci_trace( type, p_data, length );
}
#endif

static void battery_server_set_advertisement_data()
{
#if BTSTACK_VER >= 0x03000001
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);
#else
    wiced_result_t              result;
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    uint8_t ble_advertisement_flag_value        = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem                            = 0;
    uint16_t battery_server_uuid    = UUID_SERVICE_BATTERY;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &ble_advertisement_flag_value;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_16;
    adv_elem[num_elem].p_data       = (uint8_t *)&battery_server_uuid;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = app_gap_device_name_len;
    adv_elem[num_elem].p_data       = (uint8_t *)app_gap_device_name;

    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );

    WICED_BT_TRACE( "wiced_bt_ble_set_advertisement_data %d\n", result );
#endif
}

static void battery_server_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result;
    uint8_t                     *p;

    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram( BATTERY_SERVICE_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if(result == WICED_BT_SUCCESS)
    {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys, link_keys.key_data.ble_addr_type );
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db(&link_keys);
#endif
    }
    WICED_BT_TRACE("battery_server_load_keys_for_address_resolution %B result:%d \n", p, result );
}

static void battery_server_update_data(gatt_db_lookup_table_t * p_attribute)
{
    switch (p_attribute->handle)
    {
    case HDLC_BAS_BATTERY_LEVEL_VALUE:  // read battery level
        p_attribute->p_data[0]--;         // we fake battery level value by decrement each time.
        if (!p_attribute->p_data[0])
        {
            *p_attribute->p_data = MAX_BATTERY_LEVEL;
        }
        break;
    }
}

static void battery_server_send_data()
{
    gatt_db_lookup_table_t *p_attribute;
    wiced_result_t result;
    int i = battery_server_state.current_char; // index
    uint16_t handle = battery_server_state.bas_char[i].handle_val;
    uint16_t mask = 1<<i;

    if ( ( p_attribute = battery_server_get_attribute(handle) ) == NULL)
    {
        WICED_BT_TRACE("handle attr not found hdl:%x\n", handle );
        return;
    }

    battery_server_update_data(p_attribute);

#ifdef BAS_1_1
    // check if this handle is characteristics is enabled for indication
    if ( battery_server_hostinfo.indications & mask )
    {
        if (battery_server_state.indication_sent)
        {
            WICED_BT_TRACE("cannot send Battery %s (%04x) indication because the previous indication has not been confirmed\n", battery_server_state.name[i], handle);
        }
        else
        {
            result = wiced_bt_gatt_send_indication( battery_server_state.conn_id, handle, p_attribute->max_len, p_attribute->p_data );
            WICED_BT_TRACE("send Battery %s (%04x) indication, result: %d\n", battery_server_state.name[i], handle, result);
            battery_server_state.indication_sent = WICED_TRUE;
        }
    }
    else
#endif
    // check if this handle is characteristics is enabled for notification
    if ( battery_server_hostinfo.notifications & mask )
    {
        result = wiced_bt_gatt_send_notification( battery_server_state.conn_id, handle, p_attribute->max_len, p_attribute->p_data );
        WICED_BT_TRACE("send Battery %s (%04x) notification, result: %d\n", battery_server_state.name[i], handle, result);
    }
    else
    {
        WICED_BT_TRACE("notification/indication is not enabled for handle %04x\n", handle);
    }

    // update current characteristics index to next valid handle
    do
    {
        i++;
        if (i >= BAS_MAX_IDX)
        {
            i = 0;
        }

    } while (!battery_server_state.bas_char[i].handle_val);

    battery_server_state.current_char = i;
}

static void battery_server_timer_expiry_handler(  uint32_t param )
{
    battery_server_send_data();
}

static void battery_server_application_init()
{
    wiced_bt_gatt_status_t gatt_status;
#if defined(CYW20706A2) || defined(CYW20719B0)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif
    /* Register for gatt event notifications */
    gatt_status = wiced_bt_gatt_register(&battery_server_gatt_cback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /* Initialize GATT database */
    gatt_status = app_bt_gatt_db_init(GATT_DATABASE, GATT_DATABASE_LEN);

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef ENABLE_HCI_TRACE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(battery_server_hci_trace_cback);
#endif

#ifdef CYW20706A2
    /* Enable privacy to advertise with RPA */
    wiced_bt_ble_enable_privacy (WICED_TRUE);
#endif
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);


    /* Load the address resolution DB with the keys stored in the NVRAM */
    battery_server_load_keys_for_address_resolution();

    battery_server_set_advertisement_data();

    /* start LE advertising */
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("Waiting for Battery Service to connect...\n");

    wiced_init_timer(&battery_server_timer, battery_server_timer_expiry_handler, 0, WICED_SECONDS_PERIODIC_TIMER);
}

/*
 * Process SMP bonding result. If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */
static void battery_server_smp_bond_result( uint8_t result )
{
    WICED_BT_TRACE( "battery_server, bond result: %d\n", result );

    if (result == WICED_BT_SUCCESS)
    {
        uint8_t bytes_written;
        wiced_result_t nv_result = WICED_BT_SUCCESS;

        // clear flags
        battery_server_hostinfo.notifications = battery_server_hostinfo.indications = 0;

        /* Save the  host info in NVRAM  */
        bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_server_hostinfo), (uint8_t*)&battery_server_hostinfo, &nv_result );
//        WICED_BT_TRACE("NVRAM write %d, result = %d\n", bytes_written, nv_result);
        UNUSED_VARIABLE(bytes_written);
    }
}

/* based on NVRAM nofications/indications flags, populate data buffers for cccd flag reads */
static void battery_server_set_flags(void)
{
    int i;

    for (i=0; i<BAS_MAX_IDX; i++)
    {
        gatt_db_lookup_table_t *p_attribute;
        uint16_t mask = (1<<i);

        if ( ( p_attribute = battery_server_get_attribute(battery_server_state.bas_char[i].handle_cccd) ) != NULL)
        {
            p_attribute->p_data[0] = p_attribute->p_data[1] = 0; // clear flags

            // Update notification flag
            if (battery_server_hostinfo.notifications & mask)
            {
                p_attribute->p_data[0] |= GATT_CLIENT_CONFIG_NOTIFICATION;
            }

            // Update indication flag
            if (battery_server_hostinfo.indications & mask)
            {
                p_attribute->p_data[0] |= GATT_CLIENT_CONFIG_INDICATION;
            }
        }
    }
}

/*
 * Process notification from stack that encryption has been set. If connected
 * client is registered for notification or indication, it is a good time to
 * send it out
 */
static void battery_server_encryption_changed( wiced_result_t result, uint8_t* bd_addr )
{
    WICED_BT_TRACE( "encryption change bd ( %B ) res: %d \n", battery_server_hostinfo.bdaddr,  result);

    /* Connection has been encrypted meaning that we have correct/paired device
     * restore values in the database
     */
    if( result == WICED_SUCCESS )
    {
        wiced_hal_read_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_server_hostinfo), (uint8_t*)&battery_server_hostinfo, &result );

        if( result == WICED_SUCCESS )
        {
            if ( (memcmp(battery_server_hostinfo.bdaddr, battery_server_state.remote_addr, BD_ADDR_LEN) == 0) )
            {
                battery_server_set_flags();
                battery_server_check_timer();   // start timer if any flags are set
            }
        }
    }
}

/*
 * Process write request or write command from peer device
 */
static int battery_server_update_flags(int idx, uint8_t * p_val)
{
    uint16_t mask = 1<<idx;
    /* update notificataion bit */
    if( *p_val & GATT_CLIENT_CONFIG_NOTIFICATION )
    {
        battery_server_hostinfo.notifications |= mask;
    }
    else
    {
        battery_server_hostinfo.notifications &= ~mask;
    }
    WICED_BT_TRACE("notification flags: %04X", battery_server_hostinfo.notifications );

#ifdef BAS_1_1
    /* update indication bit */
    if( *p_val & GATT_CLIENT_CONFIG_INDICATION )
    {
        battery_server_hostinfo.indications |= mask;
    }
    else
    {
        battery_server_hostinfo.indications &= ~mask;
    }
    WICED_BT_TRACE(", indication flags: %04X\n", battery_server_hostinfo.indications );
#else
    WICED_BT_TRACE("\n");
#endif

    return WICED_TRUE;
}

wiced_result_t battery_server_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    uint8_t                          *p_keys;

    WICED_BT_TRACE("battery_server_management_cback: %x\n", event);

    switch(event)
    {
    /* Bluetooth stack enabled */
    case BTM_ENABLED_EVT:
        battery_server_application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        WICED_BT_TRACE("numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data   = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req   = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE( "Pairing Complete: %d",p_info->reason);
        battery_server_smp_bond_result( p_info->reason );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
        wiced_hal_write_nvram ( BATTERY_SERVICE_PAIRED_KEYS_VS_ID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("keys save to NVRAM %B result: %d \n", p_keys, result);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
        wiced_hal_read_nvram( BATTERY_SERVICE_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p_keys, &result );
        WICED_BT_TRACE("keys read from NVRAM %B result: %d \n", p_keys, result);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram ( BATTERY_SERVICE_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;


    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( BATTERY_SERVICE_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
        WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_status = &p_event_data->encryption_status;
        WICED_BT_TRACE( "Encryption Status Event: bd ( %B ) res %d \n", p_status->bd_addr, p_status->result);
        battery_server_encryption_changed( p_status->result, p_status->bd_addr );
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;

        WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
        if ( *p_mode == BTM_BLE_ADVERT_OFF )
        {
            if ( !battery_server_state.conn_id )
            {
                result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
                WICED_BT_TRACE( "wiced_bt_start_advertisements: %d\n", result );
            }
            else
            {
                WICED_BT_TRACE( "ADV stop\n");
            }
        }
        break;

    default:
        break;
    }

    return result;
}

/*
 * Find attribute description by handle
 */
gatt_db_lookup_table_t * battery_server_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i < APP_GATT_DB_EXT_ATTR_TBL_SIZE; i++ )
    {
        if ( APP_GATT_DB_EXT_ATTR_TBL[i].handle == handle )
        {
            return ( &APP_GATT_DB_EXT_ATTR_TBL[i] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}

void battery_server_check_timer()
{
    // if any notification or indication is set, we want timer running
#ifdef BAS_1_1
    if( battery_server_hostinfo.notifications | battery_server_hostinfo.indications )
#else
    if( battery_server_hostinfo.notifications )
#endif
    {
        if (!wiced_is_timer_in_use(&battery_server_timer))
        {
            wiced_start_timer( &battery_server_timer, TIMER_TICK_PERIOD_IN_SEC );
            WICED_BT_TRACE("Timer started\n");
        }
    }
    else
    {
        wiced_stop_timer( &battery_server_timer );
        WICED_BT_TRACE("Timer stopped\n");
    }
}

/*
 * Process handle write request or write command from peer device
 */
wiced_bt_gatt_status_t battery_server_gatts_write_handle(uint16_t handle, uint16_t offset, uint16_t len, uint8_t * p_data)
{
    gatt_db_lookup_table_t *p_attribute;
    uint16_t                original_notifications  = battery_server_hostinfo.notifications; // save the original notification/indication flags
    uint16_t                original_indications    = battery_server_hostinfo.indications;

    if ( ( p_attribute = battery_server_get_attribute(handle) ) == NULL)
    {
        WICED_BT_TRACE("Invalid handle %x to write\n", handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    // make sure the write is within the write limit
    if ((offset + len) > p_attribute->max_len)
    {
        WICED_BT_TRACE("offset %d + length %d is larger than buffer size %d\n", offset, len, p_attribute->max_len);
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }

//    WICED_BT_TRACE( "write handle: hdl:0x%x offset:%d len:%d\n", handle, offset, len );

    // write data
    memcpy(p_attribute->p_data + offset, p_data, len);

    // Update flags for notification/indication flags
    switch ( handle )
    {
        case HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_LEVEL_IDX, p_data);
            break;

#ifdef BATTERY_LEVEL_STATUS
        case HDLC_BAS_BATTERY_LEVEL_STATUS_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_LEVEL_STATUS_IDX, p_data);
            break;
#endif
#ifdef ESTIMATED_SERVICE_DATE
        case HDLC_BAS_ESTIMATED_SERVICE_DATE_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_ESTIMATED_SERVICE_DATE_IDX, p_data);
            break;
#endif
#ifdef BATTERY_CRITICAL_STATUS
        case HDLC_BAS_BATTERY_CRITICAL_STATUS_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_CRITICAL_STATUS_IDX, p_data);
            break;
#endif
#ifdef BATTERY_ENERGY_STATUS
        case HDLC_BAS_BATTERY_ENERGY_STATUS_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_ENERGY_STATUS_IDX, p_data);
            break;
#endif
#ifdef BATTERY_TIME_STATUS
        case HDLC_BAS_BATTERY_TIME_STATUS_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_TIME_STATUS_IDX, p_data);
            break;
#endif
#ifdef BATTERY_HEALTH_STATUS
        case HDLC_BAS_BATTERY_HEALTH_STATUS_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_HEALTH_STATUS_IDX, p_data);
            break;
#endif
#ifdef BATTERY_HEALTH_INFO
        case HDLC_BAS_BATTERY_HEALTH_INFO_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_HEALTH_INFO_IDX, p_data);
            break;
#endif
#ifdef BATTERY_INFO
        case HDLC_BAS_BATTERY_INFO_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_INFO_IDX, p_data);
            break;
#endif
#ifdef BATTERY_MANUFACTURE_NAME
        case HDLC_BAS_MANUFACTURE_NAME_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_MANUFACTURE_NAME_IDX, p_data);
            break;
#endif
#ifdef BATTERY_MANUFACTURE_NUMBER
        case HDLC_BAS_MANUFACTURE_NUMBER_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_MANUFACTURE_NUMBER_IDX, p_data);
            break;
#endif
#ifdef BATTERY_SERIAL_NUMBER
        case HDLC_BAS_SERIAL_NUMBER_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_SERIAL_NUMBER_IDX, p_data);
            break;
#endif
        default:
            break;
    }

    battery_server_check_timer();

    // if there is any flag change, we update NVRAM
    if ((original_notifications ^ battery_server_hostinfo.notifications) || (original_indications ^ battery_server_hostinfo.indications))
    {
        wiced_result_t rc;
        int bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_server_hostinfo), (uint8_t*)&battery_server_hostinfo, &rc );
//        WICED_BT_TRACE("NVRAM write:%d rc:%d\n", bytes_written, rc);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START( )
{
    int i;

    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    //wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

#ifdef BAS_1_1
    WICED_BT_TRACE( "\nBattery Service Server v1.1 Start\n\n" );
#else
    WICED_BT_TRACE( "\nBattery Service Server v1.0 Start\n\n" );
#endif

    // Register call back and configuration with stack
    app_stack_init();

    app_bas_battery_level[0] = MAX_BATTERY_LEVEL;

    battery_server_state.bas_char[BAS_BATTERY_LEVEL_IDX].handle_val = HDLC_BAS_BATTERY_LEVEL_VALUE;
#ifdef BATTERY_LEVEL_STATUS
    battery_server_state.bas_char[BAS_BATTERY_LEVEL_STATUS_IDX].handle_val = HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE;
#endif
#ifdef ESTIMATED_SERVICE_DATE
    battery_server_state.bas_char[BAS_ESTIMATED_SERVICE_DATE_IDX].handle_val = HDLC_BAS_ESTIMATED_SERVICE_DATE_VALUE;
#endif
#ifdef BATTERY_CRITICAL_STATUS
    battery_server_state.bas_char[BAS_BATTERY_CRITICAL_STATUS_IDX].handle_val = HDLC_BAS_BATTERY_CRITICAL_STATUS_VALUE;
#endif
#ifdef BATTERY_ENERGY_STATUS
    battery_server_state.bas_char[BAS_BATTERY_ENERGY_STATUS_IDX].handle_val = HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE;
#endif
#ifdef BATTERY_TIME_STATUS
    battery_server_state.bas_char[BAS_BATTERY_TIME_STATUS_IDX].handle_val = HDLC_BAS_BATTERY_TIME_STATUS_VALUE;
#endif
#ifdef BATTERY_HEALTH_STATUS
    battery_server_state.bas_char[BAS_BATTERY_HEALTH_STATUS_IDX].handle_val = HDLC_BAS_BATTERY_HEALTH_STATUS_VALUE;
#endif
#ifdef BATTERY_HEALTH_INFO
    battery_server_state.bas_char[BAS_BATTERY_HEALTH_INFO_IDX].handle_val = HDLC_BAS_BATTERY_HEALTH_INFO_VALUE;
#endif
#ifdef BATTERY_INFO
    battery_server_state.bas_char[BAS_BATTERY_INFO_IDX].handle_val = HDLC_BAS_BATTERY_INFO_VALUE;
#endif
#ifdef BATTERY_MANUFACTURE_NAME
    battery_server_state.bas_char[BAS_MANUFACTURE_NAME_IDX].handle_val = HDLC_BAS_MANUFACTURE_NAME_VALUE;
#endif
#ifdef BATTERY_MANUFACTURE_NUMBER
    battery_server_state.bas_char[BAS_MANUFACTURE_NUMBER_IDX].handle_val = HDLC_BAS_MANUFACTURE_NUMBER_VALUE;
#endif
#ifdef BATTERY_SERIAL_NUMBER
    battery_server_state.bas_char[BAS_SERIAL_NUMBER_IDX].handle_val = HDLC_BAS_SERIAL_NUMBER_VALUE;
#endif

    WICED_BT_TRACE( "Supported BAS characteristics:\n\n");

    for (i=0; i<BAS_MAX_IDX; i++)
    {
        if (battery_server_state.bas_char[i].handle_val)
        {
            WICED_BT_TRACE( "  Battery %s\n",battery_server_state.name[i]);
        }
    }

    WICED_BT_TRACE( "\n");
}


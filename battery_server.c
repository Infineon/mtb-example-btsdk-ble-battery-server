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
 * 1. Plug the AIROC eval board into your computer
 * 2. Build and download the application (to the AIROC board)
 * 3. On application start the device acts as a GATT server and advertises itself as Battery Service.
 * 4. Connect to Battery Service using one of the LE clients (LEExplorer(android)) or (BLE Utility(Apple Store))
 *    or battery_server_client application.
 * 5. Once connected the client can read Battery levels.
 * 6. If the client enables the notification, the Battery Server will send battery level every 2 secs.
 * 7. Battery Level starts from 100 and keeps decrementing to 1. The Battery Server App is designed such that,
 *    once it hits 1, it will be rolled back to 100.
 */

#include "sparcommon.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_puart.h"
#include "battery_server.h"
#include "hci_control_api.h"
#include "wiced_memory.h"
#include "wiced_bt_battery_server.h"
#include "bt_types.h"

#define TEST_HCI_CONTROL
//#define ENABLE_HCI_TRACE 1 // configures HCI traces to be routed to the AIROC HCI interface
#ifdef BATTERY_LEVEL_BROADCAST

#ifdef TEST_HCI_CONTROL
static uint32_t  battery_server_hci_rx_cmd(uint8_t *p_data, uint32_t length);
void battery_server_hci_transport_status( wiced_transport_type_t type );
static void battery_server_hci_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
static void battery_server_hci_send_disconnect_event( uint8_t reason, uint16_t con_handle );
static void battery_server_handle_char_modification_cmd( uint8_t idx );
#endif

/* Multi advertisement instance ID */
#define BATTERY_LEVEL_INSTANCE_ID 0x01

#ifdef BTSTACK_VER
#define MULTI_ADV_TX_POWER_MAX      MULTI_ADV_TX_POWER_MAX_INDEX
#endif

#if defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX) || defined(CYW55572)
/* Adv parameter used for multi-adv*/
wiced_bt_ble_multi_adv_params_t adv_param =
#else
wiced_bt_beacon_multi_advert_data_t adv_param =
#endif
{
    .adv_int_min = BTM_BLE_ADVERT_INTERVAL_MIN,
    .adv_int_max = BTM_BLE_ADVERT_INTERVAL_MAX,
    .adv_type = MULTI_ADVERT_NONCONNECTABLE_EVENT,
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,
    .adv_filter_policy = BTM_BLE_ADV_POLICY_FILTER_CONN_FILTER_SCAN,
    .adv_tx_power = MULTI_ADV_TX_POWER_MAX,
    .peer_bd_addr = {0},
    .peer_addr_type = BLE_ADDR_PUBLIC,
    .own_bd_addr = {0},
    .own_addr_type = BLE_ADDR_PUBLIC
};

#endif  // BATTERY_LEVEL_BROADCAST

#define TRANS_UART_BUFFER_SIZE          1024

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
#if BTSTACK_VER >= 0x03000001
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
#endif

#if defined(TEST_HCI_CONTROL) && defined(BATTERY_LEVEL_BROADCAST)
    .p_status_handler = battery_server_hci_transport_status,
    .p_data_handler = battery_server_hci_rx_cmd,
#else
    .p_status_handler = NULL,
    .p_data_handler = NULL,
#endif

    .p_tx_complete_cback = NULL
};



/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
/* Holds global state of the App */
battery_server_state_t battery_server_state = {
#ifdef WICED_BT_TRACE_ENABLE
    {"Level",
 #ifdef BAS_1_1
     "Level Status",
#ifdef BATTERY_LEVEL_BROADCAST
     "Level Status Broadcast",
#endif
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
#ifdef BATTERY_LEVEL_BROADCAST
     {HDLC_BAS_BATTERY_LEVEL_STATUS_SERVER_CHAR_CONFIG},
#endif
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

#ifdef BATTERY_LEVEL_BROADCAST
    if( battery_server_state.broadcast == SCC_NONE )
    {
        wiced_stop_timer( &battery_server_timer );
    }
#else
    wiced_stop_timer( &battery_server_timer );
#endif

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
#if BTSTACK_VER >= 0x03000001
    wiced_transport_send_hci_trace( type, p_data, length );
#else
    wiced_transport_send_hci_trace( NULL, type, length, p_data );
#endif
}
#endif

void battery_server_set_advertisement_data()
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

#ifdef BATTERY_LEVEL_BROADCAST

static void battery_server_set_broadcast_advertisment()
{
    WICED_BT_TRACE("battery_server_set_broadcast_advertisment()\n" );

    const uint8_t SERVICE_ADV_SIZE = 14;
    wiced_result_t result = 0;
    uint8_t service_broadcast[SERVICE_ADV_SIZE];
    uint16_t battery_server_uuid         = UUID_SERVICE_BATTERY;
    uint8_t ble_advertisement_flag_value = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    // Raw    0x0A 16 0F18 07 A304 0601 5D 00
    gatt_db_lookup_table_t *p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE );
    batt_level_status_t *p_level = ( batt_level_status_t * ) p_attribute->p_data;
    uint8_t  status_flags = p_level->flags;
    uint16_t identifier = p_level->identifier;
    uint8_t  battery_level = p_level->battery_level;

    service_broadcast[0]  = 2;
    service_broadcast[1]  = BTM_BLE_ADVERT_TYPE_FLAG;
    service_broadcast[2]  = ble_advertisement_flag_value;

    // fill adv elem with Service Data
    service_broadcast[3] = 0x0A; // len = battery level status flags(2) + Identifier(2) + ??(2) + battery level(1) + rfu(1)
    service_broadcast[4] = BTM_BLE_ADVERT_TYPE_SERVICE_DATA; //1
    memcpy(&service_broadcast[5], &battery_server_uuid, sizeof(uint16_t)); //2
    service_broadcast[7] = status_flags; // uint8_t ?? //1
    memcpy(&service_broadcast[8], &identifier, sizeof(uint16_t)); //2
    memcpy(&service_broadcast[10], &identifier, sizeof(uint16_t)); //2
    service_broadcast[12] = battery_level; //1
    service_broadcast[13] = 0x00;         // rfu 1


    // Sets adv data for multi adv instance
    result = wiced_set_multi_advertisement_data( service_broadcast, SERVICE_ADV_SIZE, BATTERY_LEVEL_INSTANCE_ID );

    WICED_BT_TRACE( "BROADCAST wiced_bt_ble_set_multi_advertisement_data %d \n", battery_level );
    UNUSED_VARIABLE(result);
}

static void battery_server_send_broadcast()
{
    wiced_result_t            result;
    static uint8_t previous_level[] = {0};

    WICED_BT_TRACE("battery_server_send_broadcast()\n");
    /*
    if ( previous_level[0] == app_bas_battery_level[0] )
    {
        WICED_BT_TRACE("battery_server_send_broadcast - no level change \n");

        result = wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BATTERY_LEVEL_INSTANCE_ID);
        WICED_BT_TRACE("Stop broadcast multi advertisements %02x\n", result);

        return;
    }
    */
    WICED_BT_TRACE("battery_server_send_broadcast - level change previous:%d current:%d\n", previous_level, app_bas_battery_level[0] );

    previous_level[0] = app_bas_battery_level[0];

    if ( battery_server_state.broadcast == SCC_NONE )
    {
        // appears broadcast is disabled since adv started, stop now
        result = wiced_start_multi_advertisements(MULTI_ADVERT_STOP, BATTERY_LEVEL_INSTANCE_ID);
        WICED_BT_TRACE("Stop broadcast multi advertisements %02x\n", result);

    } else if ( (battery_server_state.broadcast == SCC_BROADCAST) )
    {
        battery_server_set_broadcast_advertisment();

        adv_param.adv_int_min = 480;
        adv_param.adv_int_max = 480;
#if defined(CYW20735B1) || defined(CYW20835B1) || defined(CYW20819A1) || defined(CYW20719B2) || defined(CYW20721B2) || defined (WICEDX) || defined(CYW55572)
    wiced_set_multi_advertisement_params(BATTERY_LEVEL_INSTANCE_ID, &adv_param);
#else
    wiced_set_multi_advertisement_params(adv_param.adv_int_min, adv_param.adv_int_max, adv_param.adv_type,
            adv_param.own_addr_type, adv_param.own_bd_addr, adv_param.peer_addr_type, adv_param.peer_bd_addr,
            adv_param.channel_map, adv_param.adv_filter_policy,
            BATTERY_LEVEL_INSTANCE_ID, adv_param.adv_tx_power);
#endif
        result = wiced_start_multi_advertisements(MULTI_ADVERT_START, BATTERY_LEVEL_INSTANCE_ID);
        WICED_BT_TRACE("Start broadcast multi advertisements %02x\n", result);
    }
}

// updates the battery level characterstic using battery energy status
void battery_server_update_battery_level()
{

    gatt_db_lookup_table_t *p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE );
    batt_energy_status_t *p_energy = ( batt_energy_status_t * ) p_attribute->p_data;

    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE );
    batt_level_status_t *p_level = ( batt_level_status_t * ) p_attribute->p_data;

    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_INFO_VALUE );
    batt_info_t *p_binfo = ( batt_info_t * ) p_attribute->p_data;
    UINT16 available_energy = 0, available_power_capacity = 0;
    //available_power_capacity = SFLOATtoUINT16(p_energy->available_power_capacity);
    available_power_capacity = p_energy->available_power_capacity;

    if ( p_level->power_state.batt_present  == 1)
    {
        available_energy = 180; // restore this val as it may have been reset with no battery present tc.
        //p_energy->available_energy= UINT8_UINT8toSFLOAT(0, (uint8_t)available_energy, (uint8_t)0);

        /*
        available_energy -= 10; // fake a energy level change
        if ( available_energy <= low_energy ){
            available_energy = low_energy + 100; // always keep it above low energy level
            p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)available_energy, (uint8_t)00);
        }
        */
        // calculate level using current energy and energy at a full charge ( not capacity )
        p_level->battery_level = ( uint8_t )( ((100*available_energy)/available_power_capacity) );
        app_bas_battery_level[0] = p_level->battery_level;
        WICED_BT_TRACE("available_energy:%d available_power_capacity: %d, percent:%d\n", available_energy, available_power_capacity,
                        p_level->battery_level );

        p_level->power_state.batt_charge_type = 2;
        p_level->power_state.batt_charge_level = 1; // good
        p_level->power_state.batt_charge_state = 1; // charging
        //p_level->power_state.batt_charge_fault = 0; // ?
        WICED_BT_TRACE("LEVEL_VALUE battery level updated - Battery inserted/good battery level \n");

        p_attribute = battery_server_get_attribute( HDLC_BAS_MANUFACTURE_NAME_VALUE );
        memcpy( p_attribute->p_data, "Infineon Technologies", sizeof("Infineon Technologies") );

    } else if ( p_level->power_state.batt_present  == 0)
    {
        available_energy = 0; // no battery, so available energy should be zero
        //p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)available_energy, (uint8_t)00);

        // calculate level using current energy and energy at a full charge ( not capacity )
        p_level->battery_level = ( uint8_t )( ((100*available_energy)/available_power_capacity) );
        app_bas_battery_level[0] = p_level->battery_level;
        WICED_BT_TRACE("available_energy:%d available_power_capacity: %d, percent:%d\n", available_energy, available_power_capacity,
                        (available_energy*100)/available_power_capacity );

        WICED_BT_TRACE("LEVEL_VALUE battery level updated - Battery removed /Zero battery level \n");
        p_level->power_state.batt_charge_level = 0; // unknown
        p_level->power_state.batt_charge_type = 0;  // not charging
        p_level->power_state.batt_charge_state = 0; // charging

        // BAS/SR/CR/BV-TBD04-C manufacturer name set to blank
        p_attribute = battery_server_get_attribute( HDLC_BAS_MANUFACTURE_NAME_VALUE );
        memset( p_attribute->p_data, 0, p_attribute->cur_len ); // cur_len app_bas_manuf_name[] max_len
    }

    // common settings if battery is present or not.
    p_level->power_state.wired_ext_pwr_connected = 1;
    p_level->power_state.wireless_ext_pwr_connected = 0;
    p_level->power_state.rfu = 0;

    // Identifier value should match with value from characteristic Presentation Format
    p_level->identifier = 0x0001; // 0x01: first (Bluetooth SIG namespace);
    p_level->additional_status.service_required = BAS_ADDITIONAL_STATUS_SERVICE_REQUIRED_FALSE;
    p_level->additional_status.rfu = 0;

    // Also send broadcast as battery level changed
    battery_server_send_broadcast();
}

#endif // BATTERY_LEVEL_BROADCAST


static void battery_server_update_data(gatt_db_lookup_table_t * p_attribute)
{
    static  uint8_t  idx;
    idx++;

    switch (p_attribute->handle)
    {
    case HDLC_BAS_BATTERY_LEVEL_VALUE:
        p_attribute->p_data[0]--;         // we fake battery level value by decrement each time.
        if (!p_attribute->p_data[0])
        {
            *p_attribute->p_data = MAX_BATTERY_LEVEL;
        }

    /** code below is to facilitate PTS testing, can be removed after passing IOP **/
#ifdef BAS_1_1
       {
            // below also meets BAS/SR/IND/BV-TBD00-C
            static uint8_t round = 0;
            gatt_db_lookup_table_t *p_level_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE );
            batt_level_status_t  *p_level_status = (batt_level_status_t *) p_level_attribute->p_data;
            if ( round == 0 )
            {
                p_level_status->power_state.batt_present = 1; // present
                battery_server_update_battery_level();
                round = 1;

            } else if ( round == 1 )
            {
                // BAS/SR/CR/BV-TBD03-C level set to 0
                p_level_status->power_state.batt_present = 0;
                battery_server_update_battery_level();
                round = 0;
            }

            break;
        }

#endif // BAS_1_1
        break;

#ifdef BAS_1_1

    case HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE:
         {
            // For BAS_1_1 calculate latest battery level using energy status value
            // Need to modify energy status, level & critical status along with level status for BAS/SR/CR/BV-TBD01-C
            batt_level_status_t *p_level = ( batt_level_status_t * ) p_attribute->p_data;

            gatt_db_lookup_table_t *p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE );
            batt_energy_status_t *p_energy = ( batt_energy_status_t * ) p_attribute->p_data;

            p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_CRITICAL_STATUS_VALUE );
            batt_critical_status_t *p_critical = ( batt_critical_status_t * ) p_attribute->p_data;

            static uint8_t round  = 0;
            p_level->additional_status.service_required = BAS_ADDITIONAL_STATUS_SERVICE_REQUIRED_FALSE;
            p_level->additional_status.rfu = 0;

            p_level->power_state.batt_present = 1;
            p_level->power_state.wired_ext_pwr_connected = 0;
            p_level->power_state.batt_charge_level = 1;
            p_level->power_state.rfu = 0;

            if ( round ==  0 ) // step 2
            {
                // battery info = capacity(250 kwh), low energy(100) and critical(50)
                // should be greater than low energy value from battery info
                //p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)180, (uint8_t)0);
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;
                p_level->power_state.batt_charge_level = 1; // good
                p_level->power_state.wired_ext_pwr_connected = 1;

                p_critical->batt_critical_status.critical_power_state  = 0;
                p_critical->batt_critical_status.immediate_service_required = 0;
                WICED_BT_TRACE("LEVEL_STATUS_VALUE[BAS/SR/CR/BV-TBD01-C] battery level(good) %d\n", round);
                round++;
            }
            else if ( round ==  1 ) // test 4
            {
                // greater than critical energy value, but less/equal than low energy from battery info
                // p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)70, (uint8_t)0);
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;
                p_level->power_state.batt_charge_level = 2; // low

                p_critical->batt_critical_status.critical_power_state  = 0;
                p_critical->batt_critical_status.immediate_service_required = 0;
                WICED_BT_TRACE("LEVEL_STATUS_VALUE[BAS/SR/CR/BV-TBD01-C] battery level(low) %d\n", round);
                round++;
            }
            else if ( round ==  2 ) // step 6
            {
                // less than critical energy of battery info
                // p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)40, (uint8_t)0);
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;
                p_level->power_state.batt_charge_level = 3;  //critical

                p_critical->batt_critical_status.critical_power_state  = 1;
                p_critical->batt_critical_status.immediate_service_required = 0;
                WICED_BT_TRACE("LEVEL_STATUS_VALUE[BAS/SR/CR/BV-TBD01-C] battery level(critical) %d\n", round);
                round = 0;
            }

            battery_server_send_broadcast(); // level changed
            break;
        }
    case HDLC_BAS_ESTIMATED_SERVICE_DATE_VALUE:
        {
            batt_estimated_service_date_t  *p_date = (batt_estimated_service_date_t *) p_attribute->p_data;
            p_date->estimated_service_date[0] += 24; // random service date BAS/SR/IND/BV-TBD03-C
            WICED_BT_TRACE("SERVICE_DATE updated(+24)\n");
            break;
        }

    case HDLC_BAS_BATTERY_CRITICAL_STATUS_VALUE:
        {   // Rotate thru 5 different power state & additional states settings --> BAS/SR/CR/BV-TBD07-C
            static uint8_t round;
            // Need to modify both critical status and level status attributes for this test case
            batt_critical_status_t  *p_critical = (batt_critical_status_t *) p_attribute->p_data;
            gatt_db_lookup_table_t *p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE );
            batt_level_status_t *p_level = ( batt_level_status_t * ) p_attribute->p_data;

            p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE );
            batt_energy_status_t *p_energy = ( batt_energy_status_t * ) p_attribute->p_data;

            p_level->additional_status.service_required = BAS_ADDITIONAL_STATUS_SERVICE_REQUIRED_FALSE;
            p_critical->batt_critical_status.immediate_service_required = 0; // should be false if above is false
            p_critical->batt_critical_status.critical_power_state  = 0;
            p_critical->batt_critical_status.rfu = 0;

            p_level->power_state.batt_present = 1;
            p_level->additional_status.rfu = 0;
            // BAS/SR/IND/BV-TBD32-C --> critical status changed meets this requirement. Nothing else needed

            if ( round ==  0 ) // good battery level
            {
                //p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)180, (uint8_t)0); //
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;
                p_level->power_state.batt_charge_level = 1; // good
                round++;
            }
            else if ( round ==  1 ) // critical battery level
            {
                //p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)40, (uint8_t)0); // less than or equal to 50
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;

                p_level->power_state.batt_charge_level = 3;  //critical
                p_critical->batt_critical_status.critical_power_state  = 1; // true
                round++;
            }
            else if ( round ==  2 ) // low
            {
                p_level->power_state.batt_charge_level = 2;  //low
                //p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)90, (uint8_t)0); //
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;

                round++;
            }
            else if ( round ==  3 )
            {
                //p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)180, (uint8_t)0);
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;

                p_level->power_state.batt_charge_level = 1; // good, but does not matter
                round++;
            }
             else if ( round ==  4 ) // service_required == TRUE
            {
                //p_energy->available_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)180, (uint8_t)0); // less than or equal to 50
                p_level->battery_level = ( uint8_t )( ((100*p_energy->available_energy)/p_energy->available_power_capacity) );
                app_bas_battery_level[0] = p_level->battery_level;

                p_level->power_state.batt_charge_level  = 2;
                p_level->additional_status.service_required = BAS_ADDITIONAL_STATUS_SERVICE_REQUIRED_TRUE;
                p_critical->batt_critical_status.immediate_service_required = 1;
                round = 0;
            }

            battery_server_send_broadcast(); // level changed
            WICED_BT_TRACE("CRITICAL_STATUS  updated  round:%d\n", round);
            break;
        }

    case HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE:
        {
            // BAS/SR/IND/BV-TBD65-C
            // Change the value of the Power State field in the Battery Level Status
            gatt_db_lookup_table_t *p_level_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE );
            batt_level_status_t *p_level = ( batt_level_status_t * ) p_level_attribute->p_data;

            p_level->power_state.batt_present = 1;
            p_level->power_state.wired_ext_pwr_connected = 0;  // disconect battery and set it to discharging
            p_level->power_state.batt_charge_state = 2; // discharging
            p_level->power_state.rfu = 0;

            WICED_BT_TRACE("ENERGY_STATUS updated \n");
            break;
        }

    case HDLC_BAS_BATTERY_TIME_STATUS_VALUE:
        {
            batt_time_status_t *p_time = (batt_time_status_t *) p_attribute->p_data; //BAS/SR/IND/BV-TBD08-C
            p_time->time_until_discharged[0] += 10; // random time until discharge
            WICED_BT_TRACE("_TIME_STATUS time until discharge updated to +1h30m25s\n");
            break;
        }

    case HDLC_BAS_BATTERY_HEALTH_STATUS_VALUE:
        {
            batt_health_status_t *p_health = (batt_health_status_t *)p_attribute->p_data; // BAS/SR/IND/BV-TBD46-C
            p_health->health_summary += 10 ; //random. 99% = battery condition is excellent
            WICED_BT_TRACE("HEALTH_STATUS health summary increased by 10percent:%d\n", p_health->health_summary);
            break;
        }

    case HDLC_BAS_BATTERY_HEALTH_INFO_VALUE:
        {
            batt_health_info_t *p_health_info = (batt_health_info_t *) p_attribute->p_data;  //BAS/SR/IND/BV-TBD71-C
            p_health_info->cycle_count_designed_lifetime += 5;
            p_health_info->min_designed_op_temp += 2;
            p_health_info->max_designed_op_temp += 2;
            WICED_BT_TRACE("HEALTH_INFO updated (min: +2 max:+2)\n");
            break;
        }

    case HDLC_BAS_BATTERY_INFO_VALUE:
        {
            batt_info_t *p_battery_info = (batt_info_t *) p_attribute->p_data; //BAS/SR/IND/BV-TBD09-C
            p_battery_info->battery_features.batt_replaceable = 0; // not replaceable
            //p_battery_info->battery_features.batt_rechargeable = 0; // not rechargeable
            p_battery_info->battery_features.rfu = 0;

            WICED_BT_TRACE("BATTERY_INFO Battery features changed:%d \n", p_battery_info->battery_features);
            break;
        }

    case HDLC_BAS_MANUFACTURE_NAME_VALUE:
        p_attribute->p_data[0] = '1' + idx; // change first char of val to trigger indication
        WICED_BT_TRACE("BATTERY_TIME_STATUS updated idx:%d\n", idx);
        break;

    case HDLC_BAS_MANUFACTURE_NUMBER_VALUE:
        p_attribute->p_data[0] = 'A' + idx;
        WICED_BT_TRACE("MANUFACTURE_NUMBER updated idx:%d\n", idx);
        break;

    case HDLC_BAS_SERIAL_NUMBER_VALUE:
        p_attribute->p_data[0] = 'A' + idx;
        WICED_BT_TRACE("SERIAL_NUMBER updated idx:%d\n", idx);
        break;
#endif // BAS_1_1

    }
}

#ifdef BAS_1_1
static void battery_server_send_indication_or_notification( uint8_t idx )
{
     gatt_db_lookup_table_t *p_attribute;
    wiced_result_t result;
    uint16_t handle = battery_server_state.bas_char[idx].handle_val;
    uint16_t mask = 1<<idx;

    if ( ( p_attribute = battery_server_get_attribute(handle) ) == NULL)
    {
        WICED_BT_TRACE("handle attr not found hdl:%x   idx:%d \n", handle, idx);
        return;
    }

    // check if this handle is characteristics is enabled for indication
    if ( battery_server_hostinfo.indications & mask )
    {
        if (battery_server_state.indication_sent)
        {
            WICED_BT_TRACE("cannot send Battery %s (%04x) indication because the previous indication has not been confirmed\n", battery_server_state.name[idx], handle);
        }
        else
        {
            result = wiced_bt_gatt_send_indication( battery_server_state.conn_id, handle, p_attribute->max_len, p_attribute->p_data );
            WICED_BT_TRACE("send Battery %s (%04x) indication, result: %d\n", battery_server_state.name[idx], handle, result);
            battery_server_state.indication_sent = WICED_TRUE;
        }

    } else if ( battery_server_hostinfo.notifications & mask ) // check if this handle is characteristics is enabled for notification
    {
        result = wiced_bt_gatt_send_notification( battery_server_state.conn_id, handle, p_attribute->max_len, p_attribute->p_data );
        WICED_BT_TRACE("send Battery %s (%04x) notification, result: %d\n", battery_server_state.name[idx], handle, result);
    }
    else
    {
        WICED_BT_TRACE("notification/indication is not enabled for handle %04x idx:%d \n", handle, idx);
    }
}
#endif

static void battery_server_send_data()
{
    gatt_db_lookup_table_t *p_attribute;
    wiced_result_t result;
    int i = battery_server_state.current_char; // index
    uint16_t handle = battery_server_state.bas_char[i].handle_val;
    uint16_t mask = 1<<i;

    if ( ( p_attribute = battery_server_get_attribute(handle) ) == NULL)
    {
        WICED_BT_TRACE("handle attr not found hdl:%x   idx:%d \n", handle, i);
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
    // WICED_BT_TRACE("attr found for hdl:%x   idx:%d \n", handle, i);
    // check if this handle is characteristics is enabled for notification
    if ( battery_server_hostinfo.notifications & mask )
    {
        result = wiced_bt_gatt_send_notification( battery_server_state.conn_id, handle, p_attribute->max_len, p_attribute->p_data );
        WICED_BT_TRACE("send Battery %s (%04x) notification, result: %d\n", battery_server_state.name[i], handle, result);
    }
    else
    {
        WICED_BT_TRACE("notification/indication is not enabled for handle %04x idx:%d \n", handle, i);
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

static void battery_server_timer_expiry_handler(  TIMER_PARAM_TYPE param )
{
    battery_server_send_data();
}

#ifdef BATTERY_LEVEL_BROADCAST
// populate the new characterstics with fake data for test purposes
static void battery_server_characterstics_init()
{
    gatt_db_lookup_table_t *p_attribute = battery_server_get_attribute( HDLC_BAS_ESTIMATED_SERVICE_DATE_VALUE );
    batt_estimated_service_date_t *p_date = ( batt_estimated_service_date_t * ) p_attribute->p_data;
    uint8_t *buffer = p_date->estimated_service_date;
    UINT24_TO_STREAM(buffer, 20115); // service days since epoch

    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_CRITICAL_STATUS_VALUE );
    batt_critical_status_t *p_critical = ( batt_critical_status_t * ) p_attribute->p_data;
    p_critical->batt_critical_status.critical_power_state = 0;
    p_critical->batt_critical_status.immediate_service_required = 0;
    p_critical->batt_critical_status.rfu = 0;

    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE );
    batt_energy_status_t *p_energy = ( batt_energy_status_t * ) p_attribute->p_data;
    /*p_energy->external_source_power = wiced_bt_types_uint8_uint8tosfloat(0, (uint8_t)5, (uint8_t)0);
    p_energy->present_voltage = wiced_bt_types_uint8_uint8tosfloat(0, (uint8_t)4, (uint8_t)00);
    p_energy->available_energy = wiced_bt_types_uint8_uint8tosfloat(0, (uint8_t)200, (uint8_t)00);
    p_energy->available_power_capacity = wiced_bt_types_uint8_uint8tosfloat(0, (uint8_t)240, (uint8_t)00);
    p_energy->change_rate = wiced_bt_types_uint8_uint8tosfloat(0, (uint8_t)2, (uint8_t)0);
    p_energy->available_energy_at_last_change = wiced_bt_types_uint8_uint8tosfloat(0, (uint8_t)230, (uint8_t)0);
    */
    /*
    p_energy->external_source_power = UINT16toSFLOAT(5);
    p_energy->present_voltage = UINT16toSFLOAT(4);
    p_energy->available_energy = UINT16toSFLOAT(200);
    p_energy->available_power_capacity = UINT16toSFLOAT(240);
    p_energy->change_rate = UINT16toSFLOAT(2);
    p_energy->available_energy_at_last_change = UINT16toSFLOAT(230);
    */

    // must be done after energy status
    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE );
    batt_level_status_t *p_level = ( batt_level_status_t * ) p_attribute->p_data;
    p_level->power_state.batt_present = 1;
    battery_server_update_battery_level();

    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_TIME_STATUS_VALUE );
    batt_time_status_t *p_time = ( batt_time_status_t * ) p_attribute->p_data;
    p_time->time_until_discharged[0] = 55; // minutes
    p_time->time_until_discharged_on_standby[0] = 94;
    p_time->time_until_recharged[0] = 25;

    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_HEALTH_STATUS_VALUE );
    batt_health_status_t *p_health = ( batt_health_status_t * ) p_attribute->p_data;
    p_health->health_summary = 80;
    p_health->cycle_count = 800;
    p_health->current_temp = 80;
    p_health->deep_discharge_count = 24;

    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_HEALTH_INFO_VALUE );
    batt_health_info_t *p_hinfo = ( batt_health_info_t * ) p_attribute->p_data;
    p_hinfo->cycle_count_designed_lifetime = 300;
    p_hinfo->min_designed_op_temp = 10;
    p_hinfo->max_designed_op_temp = 55;

    // Battery info is static, except for battery features
    p_attribute = battery_server_get_attribute( HDLC_BAS_BATTERY_INFO_VALUE );
    batt_info_t *p_binfo = ( batt_info_t * ) p_attribute->p_data;
    p_binfo->battery_features.batt_replaceable = 1; // replaceable
    p_binfo->battery_features.batt_rechargeable = 1; //rechargeable
    p_binfo->battery_features.rfu = 0;

    p_binfo->batt_manuf_date[0] = 0x34; p_binfo->batt_manuf_date[1] = 0x4A;
    p_binfo->batt_expr_date[0] = 0xC0; p_binfo->batt_expr_date[1] = 0x52;
    //p_binfo->batt_designed_cap = UINT8_UINT8toSFLOAT(0, (uint8_t)250, (uint8_t)00);
    //p_binfo->batt_low_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)100, (uint8_t)00);
    //p_binfo->batt_critical_energy = UINT8_UINT8toSFLOAT(0, (uint8_t)50, (uint8_t)00);

    p_binfo->batt_chemistry = 11;
    p_binfo->batt_nominal_volage = 317;
    p_binfo->batt_aggr_grp = 32;
}
#endif


static void battery_server_application_init()
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result;
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

#ifdef BATTERY_LEVEL_BROADCAST

    battery_server_characterstics_init();
    // broadcast of battery level previously enabled?
    wiced_hal_read_nvram( BATTERY_SERVICE_BROADCAST_VS_ID, sizeof(battery_server_state.broadcast), (uint8_t*)&battery_server_state.broadcast, &result );
    if( result != WICED_SUCCESS )
    {
        battery_server_state.broadcast = SCC_NONE;
    }
    battery_server_check_timer();
#endif

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

        /* Save the  host info in NVRAM  */
        bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_server_hostinfo), (uint8_t*)&battery_server_hostinfo, &nv_result );
        WICED_BT_TRACE("NVRAM write %d, result = %d. Notification/Indication flags cleared \n", bytes_written, nv_result);
        UNUSED_VARIABLE(bytes_written);
    }
}

/* based on NVRAM notifications/indications flags, populate data buffers for cccd flag reads */
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
    WICED_BT_TRACE( "battery_server_set_flags   broadcast: %d \n", battery_server_state.broadcast );
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

#ifdef BATTERY_LEVEL_BROADCAST
                WICED_BT_TRACE( "<-Found a known device......................-> \n");
                // for PTS testing(BAS/SR/IND/TBD73-C), fake a battery level change and send an indication
                battery_server_handle_char_modification_cmd( BAS_BATTERY_LEVEL_STATUS_IDX );
                battery_server_send_indication_or_notification( BAS_BATTERY_LEVEL_STATUS_IDX );

                // Following send notify/indications needed after a reconnection with a bonded client
                battery_server_send_indication_or_notification( BAS_BATTERY_ENERGY_STATUS_IDX );
                battery_server_send_indication_or_notification( BAS_BATTERY_TIME_STATUS_IDX );
                battery_server_send_indication_or_notification( BAS_BATTERY_HEALTH_STATUS_IDX );
                battery_server_send_broadcast();
#endif
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
    WICED_BT_TRACE("notification idx:%d   flags: %04X\n", idx, battery_server_hostinfo.notifications );

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
    WICED_BT_TRACE(", indication idx:%d    flags: %04X\n", idx, battery_server_hostinfo.indications );
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
    if( battery_server_hostinfo.notifications | battery_server_hostinfo.indications
                |  battery_server_state.broadcast )
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
    wiced_result_t rc;
    int bytes_written;

#ifdef BATTERY_LEVEL_BROADCAST
    uint8_t                 original_broadcast      = battery_server_state.broadcast; // and broadcast as well
#endif

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

   WICED_BT_TRACE( "write handle: hdl:0x%x offset:%d len:%d\n", handle, offset, len );

    // write data
    memcpy(p_attribute->p_data + offset, p_data, len);

    // Update flags for notification/indication flags
    switch ( handle )
    {
        case HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG:
            battery_server_update_flags(BAS_BATTERY_LEVEL_IDX, p_data);
            break;

#ifdef BATTERY_LEVEL_BROADCAST
        case HDLC_BAS_BATTERY_LEVEL_STATUS_SERVER_CHAR_CONFIG:
            if( *p_data == GATT_SERVER_CONFIG_BROADCAST )
            {
                battery_server_state.broadcast = SCC_BROADCAST;

            } else if ( *p_data == GATT_SERVER_CONFIG_NONE )
            {
                battery_server_state.broadcast = SCC_NONE;
            }
            WICED_BT_TRACE("battery level broadcast -------------------->> flag: %02X \n", battery_server_state.broadcast );
            break;
#endif

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
#ifdef SECOND_BATTERY
        case HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG2:
            battery_server_update_flags(BAS_BATTERY_LEVEL_IDX2, p_data);
            break;
#endif
        default:
            break;
    }

    battery_server_check_timer();

    // if there is any flag change, we update NVRAM
    if ((original_notifications ^ battery_server_hostinfo.notifications) || (original_indications ^ battery_server_hostinfo.indications))
    {
         bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_server_hostinfo), (uint8_t*)&battery_server_hostinfo, &rc );
//        WICED_BT_TRACE("NVRAM write:%d rc:%d\n", bytes_written, rc);
    }

#ifdef BATTERY_LEVEL_BROADCAST
    if ( original_broadcast ^ battery_server_state.broadcast )
    {
       bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_BROADCAST_VS_ID,
                                sizeof(battery_server_state.broadcast), (uint8_t*)&battery_server_state.broadcast, &rc );
       WICED_BT_TRACE("NVRAM broadcast flag write:%d rc:%d\n", bytes_written, rc);
    }
#endif

    UNUSED_VARIABLE(bytes_written);
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

#ifdef ENABLE_HCI_TRACE
    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
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
    // if there is any flag change, we update NVRAM
#ifdef BATTERY_LEVEL_STATUS
    battery_server_state.bas_char[BAS_BATTERY_LEVEL_STATUS_IDX].handle_val = HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE;
#endif
#ifdef BATTERY_LEVEL_BROADCAST
    battery_server_state.bas_char[BAS_BATTERY_LEVEL_STATUS_BROADCAST_IDX].handle_val = HDLC_BAS_BATTERY_LEVEL_STATUS_SERVER_CHAR_CONFIG;
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
#ifdef SECOND_BATTERY
    battery_server_state.bas_char[BAS_BATTERY_LEVEL_IDX2].handle_val = HDLC_BAS_BATTERY_LEVEL_VALUE2;
#endif
    WICED_BT_TRACE( "Supported BAS characteristics:\n\n");

    for (i=0; i<BAS_MAX_IDX; i++)
    {
        if (battery_server_state.bas_char[i].handle_val)
        {
            WICED_BT_TRACE( "  Battery idx:%d----Name:%s handle_ccd:%02X handle_val:%02X \n",i, battery_server_state.name[i],
                                        battery_server_state.bas_char[i].handle_cccd, battery_server_state.bas_char[i].handle_val);
        }
    }

    WICED_BT_TRACE( "\n");
}

#if defined(TEST_HCI_CONTROL) && defined(BATTERY_LEVEL_BROADCAST)
/*
 *  transfer connection event to uart
 */
void battery_server_hci_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role )
{
    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = role;

    wiced_transport_send_data ( HCI_CONTROL_BATT_CLIENT_EVENT_CONNECTED, tx_buf, ( int )( p - tx_buf ) );
}


void battery_server_hci_handle_get_version(void)
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;
    uint8_t  addr[]    = {0x0A, 0X0B, 0xC, 0x0D, 0x0E, 0X0F};

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819

#if (CHIP==CHIP_20819 || CHIP==CHIP_20820 )
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_BATT_CLIENT;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);


    // if connected already, simply send connection details. okay to use fake values
    battery_server_hci_send_connect_event( 1, battery_server_hostinfo.bdaddr, battery_server_state.conn_id, 1 );

}

/* transport status */
void battery_server_hci_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE("battery_server_transport connected type: %d ", type);
}

/*
 *  transfer disconnection event to UART
 */
void battery_server_hci_send_disconnect_event( uint8_t reason, uint16_t con_handle )
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = reason;

    wiced_transport_send_data ( HCI_CONTROL_BATT_CLIENT_EVENT_DISCONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

static wiced_bt_gatt_status_t  battery_server_hci_handle_broadcast_modify( wiced_bool_t enable )
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;
    wiced_result_t rc;

    WICED_BT_TRACE("HCI Control broadcast cmd broadcast:%s\n", enable ? "On" : "Off");

    battery_server_state.broadcast = (enable == WICED_TRUE ) ? SCC_BROADCAST : SCC_NONE;
    uint8_t bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_BROADCAST_VS_ID,
                                sizeof(battery_server_state.broadcast), (uint8_t*)&battery_server_state.broadcast, &rc );
    WICED_BT_TRACE("NVRAM broadcast flag write:%d rc:%d\n", bytes_written, rc);

    return gatt_status;
}

void battery_server_handle_char_modification_cmd( uint8_t idx )
{
    gatt_db_lookup_table_t * p_attribute;
    uint16_t handle = battery_server_state.bas_char[idx].handle_val;

    if ( ( p_attribute = battery_server_get_attribute( handle ) ) == NULL)
    {
        WICED_BT_TRACE("handle attr not found hdl:%x   idx:%d \n", handle, idx);
        return;
    }

    WICED_BT_TRACE("handle attr found hdl:%x   idx:%d \n", handle, idx);
    battery_server_update_data( p_attribute );
    battery_server_send_indication_or_notification( idx );
}

uint32_t  battery_server_hci_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t                opcode;
    uint8_t*                p_data = p_buffer;
    uint16_t                payload_len;
    uint8_t                 status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;
    uint8_t  addr[]         = {0x0A, 0X0B, 0xC, 0x0D, 0x0E, 0X0F};


    WICED_BT_TRACE("hci_control_proc_rx_cmd:%d\n", length);

    if ( !p_data )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if (length < 4)
    {
        WICED_BT_TRACE( "invalid params\n" );
        wiced_transport_free_buffer( p_data );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    else
    {
        STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
        STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length
        WICED_BT_TRACE("cmd_opcode 0x%02x payload_len %d \n", opcode, payload_len);

        switch ( opcode )
        {

            case HCI_CONTROL_BATT_CLIENT_COMMAND_CONNECT:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_CONNECT\n");
                // if connected already, simply send connection details. okay to use fake values
                //battery_server_hci_send_connect_event( 1, addr, battery_server_state.conn_id, 1 );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_DISCONNECT:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_LEVEL_MODIFY\n");
                break;
            /*
            case HCI_CONTROL_BATT_CLIENT_COMMAND_START_ADV:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_MODIFY_LEVEL_STATUS\n");
                battery_server_handle_char_modification_cmd( BAS_BATTERY_LEVEL_IDX );
                battery_server_send_indication_or_notification( BAS_BATTERY_LEVEL_STATUS_IDX );
                break;
            */
            case HCI_CONTROL_BATT_CLIENT_COMMAND_ENABLE_BROADCAST:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_ENABLE_BROADCAST\n");
                battery_server_hci_handle_broadcast_modify( WICED_TRUE );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_DISABLE_BROADCAST:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_DISABLE_BROADCAST\n");
                battery_server_hci_handle_broadcast_modify( WICED_FALSE );
                break;

            // Battery Level Status
            case HCI_CONTROL_BATT_CLIENT_COMMAND_LEVEL_STATUS_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_LEVEL_STATUS_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_BATTERY_LEVEL_STATUS_IDX );
                battery_server_send_indication_or_notification( BAS_BATTERY_ENERGY_STATUS_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_LEVEL_STATUS_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_LEVEL_STATUS_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_BATTERY_LEVEL_STATUS_IDX );
                break;

            // Estimated Service Data
            case HCI_CONTROL_BATT_CLIENT_COMMAND_SERVICE_DATE_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_ESTIMATED_SERVICE_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_ESTIMATED_SERVICE_DATE_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_SERVICE_DATE_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_ESTIMATED_SERVICE_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_ESTIMATED_SERVICE_DATE_IDX );
                break;

           // Battery critical status
            case HCI_CONTROL_BATT_CLIENT_COMMAND_CRITICAL_STATUS_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_CRITICAL_STATUS_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_BATTERY_CRITICAL_STATUS_IDX );
                battery_server_send_indication_or_notification( BAS_BATTERY_LEVEL_STATUS_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_CRITICAL_STATUS_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_CRITICAL_STATUS_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_BATTERY_CRITICAL_STATUS_IDX );
                break;

            // Battery energy status
            case HCI_CONTROL_BATT_CLIENT_COMMAND_ENERGY_STATUS_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_ENERGY_STATUS_MODIFY\n");
                // BAS/SR/IND/TBD65-C needs updating battery level status and energy status
                battery_server_handle_char_modification_cmd( BAS_BATTERY_LEVEL_STATUS_IDX );
                battery_server_handle_char_modification_cmd( BAS_BATTERY_ENERGY_STATUS_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_ENERGY_STATUS_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_ENERGY_STATUS_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_BATTERY_ENERGY_STATUS_IDX );
                break;

            // Battery Time status
            case HCI_CONTROL_BATT_CLIENT_COMMAND_TIME_STATUS_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_TIME_STATUS_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_BATTERY_TIME_STATUS_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_TIME_STATUS_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_TIME_STATUS_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_BATTERY_TIME_STATUS_IDX );
                break;

            // Battery health status
            case HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_STATUS_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_STATUS_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_BATTERY_HEALTH_STATUS_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_STATUS_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_STATUS_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_BATTERY_HEALTH_STATUS_IDX );
                break;

            // Battery health info
            case HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_INFO_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_INFO_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_BATTERY_HEALTH_INFO_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_INFO_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_HEALTH_INFO_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_BATTERY_HEALTH_INFO_IDX );
                break;

            // Battery Info
            case HCI_CONTROL_BATT_CLIENT_COMMAND_BATTERY_INFO_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_BATTERY_INFO_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_BATTERY_INFO_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_BATTERY_INFO_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_BATTERY_INFO_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_BATTERY_INFO_IDX );
                break;

            // Manufacturer name
            case HCI_CONTROL_BATT_CLIENT_COMMAND_NAME_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_NAME_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_MANUFACTURE_NAME_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_NAME_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_NAME_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_MANUFACTURE_NAME_IDX );
                break;

            // Model
            case HCI_CONTROL_BATT_CLIENT_COMMAND_MODEL_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_MODEL_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_MANUFACTURE_NUMBER_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_MODEL_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_MODEL_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_MANUFACTURE_NUMBER_IDX );
                break;

            // serial Number
            case HCI_CONTROL_BATT_CLIENT_COMMAND_NUMBER_MODIFY:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_NUMBER_MODIFY\n");
                battery_server_handle_char_modification_cmd( BAS_SERIAL_NUMBER_IDX );
                break;

            case HCI_CONTROL_BATT_CLIENT_COMMAND_NUMBER_SIGNAL:
                WICED_BT_TRACE("HCI_CONTROL_BATT_CLIENT_COMMAND_NUMBER_SIGNAL\n");
                battery_server_send_indication_or_notification( BAS_SERIAL_NUMBER_IDX );
                break;

            case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
                battery_server_hci_handle_get_version();
                break;

            default:
                WICED_BT_TRACE("ignored opcode:%02X payload_len:%d\n", opcode, payload_len);
                status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
                break;
            }
    }

    wiced_transport_send_data(HCI_CONTROL_BATT_CLIENT_EVENT_STATUS, &status, 1);

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_buffer );
    return HCI_CONTROL_STATUS_SUCCESS;
}

#endif

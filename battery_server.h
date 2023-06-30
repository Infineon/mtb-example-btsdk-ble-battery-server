/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Battery Service Server Header file
 *
 */
#ifndef _BATTERY_SERVER_H_
#define _BATTERY_SERVER_H_

#include "cycfg_gatt_db.h"
#ifdef BAS_1_1
#include "cycfg_gatt_db_1_1.h"
#endif
#include "wiced_transport.h"
#include "wiced_timer.h"
#include "wiced_bt_cfg.h"
#include "app.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

#ifdef BAS_1_1
 #define GATT_DATABASE                  gatt_database_1_1
 #define GATT_DATABASE_LEN              gatt_database_len_1_1
 #define APP_GATT_DB_EXT_ATTR_TBL       app_gatt_db_ext_attr_tbl_1_1
 #define APP_GATT_DB_EXT_ATTR_TBL_SIZE  app_gatt_db_ext_attr_tbl_size_1_1
#else
 #define GATT_DATABASE                  gatt_database
 #define GATT_DATABASE_LEN              gatt_database_len
 #define APP_GATT_DB_EXT_ATTR_TBL       app_gatt_db_ext_attr_tbl
 #define APP_GATT_DB_EXT_ATTR_TBL_SIZE  app_gatt_db_ext_attr_tbl_size
#endif


typedef enum {
   BAS_BATTERY_LEVEL_IDX,
   BAS_BATTERY_LEVEL_STATUS_IDX,
   BAS_BATTERY_LEVEL_STATUS_BROADCAST_IDX,
   BAS_ESTIMATED_SERVICE_DATE_IDX,
   BAS_BATTERY_CRITICAL_STATUS_IDX,
   BAS_BATTERY_ENERGY_STATUS_IDX,
   BAS_BATTERY_TIME_STATUS_IDX,
   BAS_BATTERY_HEALTH_STATUS_IDX,
   BAS_BATTERY_HEALTH_INFO_IDX,
   BAS_BATTERY_INFO_IDX,
   BAS_MANUFACTURE_NAME_IDX,
   BAS_MANUFACTURE_NUMBER_IDX,
   BAS_SERIAL_NUMBER_IDX,
#ifdef SECOND_BATTERY
   BAS_BATTERY_LEVEL_IDX2,
#endif
   BAS_MAX_IDX,
} bas_service_idx_e;

#define BAS_BATTERY_LEVEL_BIT           (1<<BAS_BATTERY_LEVEL_IDX)
#define BAS_BATTERY_LEVEL_STATUS_BIT    (1<<BAS_BATTERY_LEVEL_STATUS_IDX)
#define BAS_ESTIMATED_SERVICE_DATE_BIT  (1<<BAS_ESTIMATED_SERVICE_DATE_IDX)
#define BAS_BATTERY_CRITICAL_STATUS_BIT (1<<BAS_BATTERY_CRITICAL_STATUS_IDX)
#define BAS_BATTERY_ENERGY_STATUS_BIT   (1<<BAS_BATTERY_ENERGY_STATUS_IDX)
#define BAS_BATTERY_TIME_STATUS_BIT     (1<<BAS_BATTERY_TIME_STATUS_IDX)
#define BAS_BATTERY_HEALTH_STATUS_BIT   (1<<BAS_BATTERY_HEALTH_STATUS_IDX)
#define BAS_BATTERY_HEALTH_INFO_BIT     (1<<BAS_BATTERY_HEALTH_INFO_IDX)
#define BAS_BATTERY_INFO_BIT            (1<<BAS_BATTERY_INFO_IDX)
#define BAS_MANUFACTURE_NAME_BIT        (1<<BAS_MANUFACTURE_NAME_IDX)
#define BAS_MANUFACTURE_NUMBER_BIT      (1<<BAS_MANUFACTURE_NUMBER_IDX)
#define BAS_SERIAL_NUMBER_BIT           (1<<BAS_SERIAL_NUMBER_IDX)
#ifdef SECOND_BATTERY
#define BAS_BATTERY_LEVEL_BIT2          (1<<BAS_BATTERY_LEVEL_IDX2)
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define BATTERY_SERVICE_GATTS_MAX_CONN              1
#define MAX_BATTERY_LEVEL                         100
#define BATTERY_SERVICE_VS_ID              WICED_NVRAM_VSID_START
#define BATTERY_SERVICE_LOCAL_KEYS_VS_ID   ( BATTERY_SERVICE_VS_ID + 1 )
#define BATTERY_SERVICE_PAIRED_KEYS_VS_ID  BATTERY_SERVICE_LOCAL_KEYS_VS_ID + 1
#define BATTERY_SERVICE_BROADCAST_VS_ID    BATTERY_SERVICE_PAIRED_KEYS_VS_ID + 1
#define TIMER_TICK_PERIOD_IN_SEC           2    // get call back every 2 sec.

/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    uint16_t  handle_cccd;                 // cccd handle
    uint16_t  handle_val;                  // char value handle
} bas_char_t;

typedef struct
{
#ifdef WICED_BT_TRACE_ENABLE
    char *    name[BAS_MAX_IDX];
#endif
    bas_char_t bas_char[BAS_MAX_IDX];
    BD_ADDR   remote_addr;              // remote peer device address
    uint16_t  conn_id;                  // connection ID referenced by the stack
    uint8_t   current_char;             // current char.
    uint8_t   indication_sent;
    uint8_t   broadcast;               // status of battery level broadcast
}battery_server_state_t;

#pragma pack(1)

/* Host information saved in  NVRAM */
typedef struct
{
    BD_ADDR  bdaddr;                               /* BD address of the bonded host */
    uint16_t notifications;  /* Current value of notification flags */
    uint16_t indications;  /* Current value of indication flags */
} host_info_t;

#pragma pack()

#ifdef BATTERY_LEVEL_BROADCAST
//Server Characteristic Configuration mode
enum bas_scc_mode
{
    SCC_NONE      = 0x00,
    SCC_BROADCAST = 0x01,
};
#endif


/******************************************************************************
 *                             External data
 ******************************************************************************/
extern uint8_t app_bas_battery_level[];
extern uint8_t app_bas_battery_level_client_char_config[];
extern uint8_t app_bas_battery_level_server_char_config[];
extern uint8_t app_bas_battery_level_status_client_char_config[];
extern uint8_t app_bas_estimated_service_date_client_char_config[];
extern uint8_t app_bas_critical_status_client_char_config[];
extern uint8_t app_bas_energy_status_client_char_config[];
extern uint8_t app_bas_time_status_client_char_config[];
extern uint8_t app_bas_health_status_client_char_config[];
extern uint8_t app_bas_health_info_client_char_config[];
extern uint8_t app_bas_info_client_char_config[];
extern uint8_t app_bas_manuf_name_client_char_config[];
extern uint8_t app_bas_manuf_num_client_char_config[];
extern uint8_t app_bas_serial_num_client_char_config[];
#ifdef SECOND_BATTERY
extern uint8_t app_bas_battery_level2;
extern uint8_t app_bas_battery_level_client_char_config2[];
#endif

extern battery_server_state_t battery_server_state;
extern host_info_t battery_server_hostinfo;
extern wiced_timer_t battery_server_timer;
extern const wiced_transport_cfg_t  transport_cfg;
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************************************
 *                             Function prototyping
 ******************************************************************************/
gatt_db_lookup_table_t * battery_server_get_attribute( uint16_t handle );
void battery_server_check_timer();
wiced_bt_gatt_status_t battery_server_gatts_req_cb(wiced_bt_gatt_attribute_request_t *p_data);
wiced_bt_gatt_status_t battery_server_gatts_write_handle(uint16_t handle, uint16_t offset, uint16_t len, uint8_t * p_data);
wiced_result_t battery_server_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
wiced_result_t app_stack_init();
wiced_bt_gatt_status_t battery_server_gatt_cback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t battery_server_gatts_conn_status_cb(wiced_bt_gatt_connection_status_t *p_status);
void battery_server_set_advertisement_data();

#endif // _BATTERY_SERVER_H_

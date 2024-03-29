/*
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
/***************************************************************************//**
* File Name: cycfg_gatt_db_1_1.c
*
* Description:
* LE device's GATT database and device configuration.
* This file should not be modified. It was automatically generated by
* Bluetooth Configurator 2.50.0.5679
*
********************************************************************************
* Copyright 2021 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifdef BAS_1_1

#include "cycfg_gatt_db.h"
#include "cycfg_gatt_db_1_1.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_battery_server.h"

/*************************************************************************************
* GATT server definitions
*************************************************************************************/
#ifdef LIFE_CRITICAL
#define LEGATTDB_CHAR_PROP_OPTION (LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE)
#else
#define LEGATTDB_CHAR_PROP_OPTION LEGATTDB_CHAR_PROP_NOTIFY
#endif

const uint8_t gatt_database_1_1[] =
{
    /* Primary Service: Generic Access */
    PRIMARY_SERVICE_UUID16 (HDLS_GAP, __UUID_SERVICE_GENERIC_ACCESS),
        /* Characteristic: Device Name */
        CHARACTERISTIC_UUID16 (HDLC_GAP_DEVICE_NAME, HDLC_GAP_DEVICE_NAME_VALUE, __UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),
        /* Characteristic: Appearance */
        CHARACTERISTIC_UUID16 (HDLC_GAP_APPEARANCE, HDLC_GAP_APPEARANCE_VALUE, __UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

    /* Primary Service: Generic Attribute */
    PRIMARY_SERVICE_UUID16 (HDLS_GATT, __UUID_SERVICE_GENERIC_ATTRIBUTE),

    /* Primary Service: Battery */
    PRIMARY_SERVICE_UUID16 (HDLS_BAS, __UUID_SERVICE_BATTERY),
        /* Characteristic: Battery Level */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_LEVEL, HDLC_BAS_BATTERY_LEVEL_VALUE, __UUID_CHARACTERISTIC_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#ifdef SECOND_BATTERY
            CHAR_DESCRIPTOR_UUID16 (HDLD_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT, LEGATTDB_PERM_READABLE),
#endif
#ifdef BATTERY_LEVEL_STATUS
        /* Characteristic: Battery Level Status */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_LEVEL_STATUS, HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE, __UUID_CHARACTERISTIC_BATTERY_LEVEL_STATUS, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_OPTION, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_LEVEL_STATUS_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

#ifdef BATTERY_LEVEL_BROADCAST
            /* Descriptor: Server Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_LEVEL_STATUS_SERVER_CHAR_CONFIG, UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE),
#endif

#endif
#ifdef ESTIMATED_SERVICE_DATE
/* Characteristic Battery Estimated Service Date */
        CHARACTERISTIC_UUID16 (HDLC_BAS_ESTIMATED_SERVICE_DATE, HDLC_BAS_ESTIMATED_SERVICE_DATE_VALUE, __UUID_CHARACTERISTIC_BATTERY_ESTIMATED_SERVICE_DATE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_OPTION, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_ESTIMATED_SERVICE_DATE_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_CRITICAL_STATUS
/* Characteristic Battery Critical Status */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_CRITICAL_STATUS, HDLC_BAS_BATTERY_CRITICAL_STATUS_VALUE, __UUID_CHARACTERISTIC_BATTERY_CRITICAL_STATUS, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_CRITICAL_STATUS_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_ENERGY_STATUS
/* Characteristic Battery Energy Status */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_ENERGY_STATUS, HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE, __UUID_CHARACTERISTIC_BATTERY_ENERGY_STATUS, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_OPTION, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_ENERGY_STATUS_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_TIME_STATUS
/* Characteristic Battery Time Status */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_TIME_STATUS, HDLC_BAS_BATTERY_TIME_STATUS_VALUE, __UUID_CHARACTERISTIC_BATTERY_TIME_STATUS, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_OPTION, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_TIME_STATUS_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_HEALTH_STATUS
/* Characteristic Battery Health Status */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_HEALTH_STATUS, HDLC_BAS_BATTERY_HEALTH_STATUS_VALUE, __UUID_CHARACTERISTIC_BATTERY_HEALTH_STATUS, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_OPTION, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_HEALTH_STATUS_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_HEALTH_INFO
/* Characteristic Battery Health Information */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_HEALTH_INFO, HDLC_BAS_BATTERY_HEALTH_INFO_VALUE, __UUID_CHARACTERISTIC_BATTERY_HEALTH_INFO, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_HEALTH_INFO_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_INFO
/* Characteristic Battery Information */
        CHARACTERISTIC_UUID16 (HDLC_BAS_BATTERY_INFO, HDLC_BAS_BATTERY_INFO_VALUE, __UUID_CHARACTERISTIC_BATTERY_INFO, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_BATTERY_INFO_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_MANUFACTURE_NAME
/* Characteristic Battery Manufacture Name */
        CHARACTERISTIC_UUID16 (HDLC_BAS_MANUFACTURE_NAME, HDLC_BAS_MANUFACTURE_NAME_VALUE, __UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NAME, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_MANUFACTURE_NAME_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_MANUFACTURE_NUMBER
/* Characteristic Battery Manufacture Number */
        CHARACTERISTIC_UUID16 (HDLC_BAS_MANUFACTURE_NUMBER, HDLC_BAS_MANUFACTURE_NUMBER_VALUE, __UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NUMBER, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_MANUFACTURE_NUMBER_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif
#ifdef BATTERY_SERIAL_NUMBER
/* Characteristic Battery Serial Number */
        CHARACTERISTIC_UUID16 (HDLC_BAS_SERIAL_NUMBER, HDLC_BAS_SERIAL_NUMBER_VALUE, __UUID_CHARACTERISTIC_BATTERY_SERIAL_NUMBER, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLC_BAS_SERIAL_NUMBER_CLIENT_CHAR_CONFIG, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif

#ifdef SECOND_BATTERY
    /* Primary Service: Battery */
    PRIMARY_SERVICE_UUID16(HDLS_BAS2, __UUID_SERVICE_BATTERY),
        /* Characteristic: Battery Level */
        CHARACTERISTIC_UUID16(HDLC_BAS_BATTERY_LEVEL2, HDLC_BAS_BATTERY_LEVEL_VALUE2, __UUID_CHARACTERISTIC_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE(HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG2, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),
            CHAR_DESCRIPTOR_UUID16(HDLD_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT2, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT, LEGATTDB_PERM_READABLE),
#endif
};

/* Length of the GATT database */
const uint16_t gatt_database_len_1_1 = sizeof(gatt_database_1_1);

/*************************************************************************************
 * GATT Initial Value Arrays
 ************************************************************************************/
/* Characteristic Presentation Format: 0x04: unsigned 8 bit integer; 0x00: no exponent; 0x2B04: unit = percentage;
   0x01: first (Bluetooth SIG namespace); 0x0000: No description */
uint8_t                         app_bas_battery_level_presentation_format[]   = {0x04, 0x00, 0x04, 0x2B, 0x01, 0x00, 0x00};

#ifdef BATTERY_LEVEL_STATUS
batt_level_status_t             app_bas_battery_level_status = {BATTERY_LEVEL_STATUS_FLAGS};
uint8_t                         app_bas_battery_level_status_client_char_config[]   = {0x00, 0x00, };

#ifdef BATTERY_LEVEL_BROADCAST
uint8_t app_bas_battery_level_status_server_char_config[] = {0x00, 0x00, };
const uint16_t app_bas_battery_level_status_server_char_config_len = (sizeof(app_bas_battery_level_status_server_char_config));
#endif

#endif
#ifdef ESTIMATED_SERVICE_DATE
batt_estimated_service_date_t   app_bas_estimated_service_date;
uint8_t                         app_bas_estimated_service_date_client_char_config[] = {0x00, 0x00, };
#endif
#ifdef BATTERY_CRITICAL_STATUS
batt_critical_status_t          app_bas_critical_status;
uint8_t                         app_bas_critical_status_client_char_config[]= {0x00, 0x00, };
#endif
#ifdef BATTERY_ENERGY_STATUS
batt_energy_status_t            app_bas_energy_status = {BATTERY_ENERGY_STATUS_FLAG};
uint8_t                         app_bas_energy_status_client_char_config[]  = {0x00, 0x00, };
#endif
#ifdef BATTERY_TIME_STATUS
batt_time_status_t              app_bas_time_status = {BATTERY_TIME_STATUS_FLAG};
uint8_t                         app_bas_time_status_client_char_config[]    = {0x00, 0x00, };
#endif
#ifdef BATTERY_HEALTH_STATUS
batt_health_status_t            app_bas_health_status = {BATTERY_HEALTH_STATUS_FLAG};
uint8_t                         app_bas_health_status_client_char_config[]  = {0x00, 0x00, };
#endif
#ifdef BATTERY_HEALTH_INFO
batt_health_info_t              app_bas_health_info = {BATTERY_HEALTH_INFO_FLAG};
uint8_t                         app_bas_health_info_client_char_config[]    = {0x00, 0x00, };
#endif
#ifdef BATTERY_INFO
batt_info_t                     app_bas_info = {BATTERY_INFO_FLAG};
uint8_t                         app_bas_info_client_char_config[]           = {0x00, 0x00, };
#endif
#ifdef BATTERY_MANUFACTURE_NAME
uint8_t                 app_bas_manuf_name[]                        = "Infineon Technologies";
uint8_t                 app_bas_manuf_name_client_char_config[]     = {0x00, 0x00, };
#endif
#ifdef BATTERY_MANUFACTURE_NUMBER
uint8_t                 app_bas_manuf_num[]                         = "0123456789";
uint8_t                 app_bas_manuf_num_client_char_config[]      = {0x00, 0x00, };
#endif
#ifdef BATTERY_SERIAL_NUMBER
uint8_t                 app_bas_serial_num[]                        = "9876543210";
uint8_t                 app_bas_serial_num_client_char_config[]     = {0x00, 0x00, };
#endif
#ifdef SECOND_BATTERY
uint8_t                 app_bas_battery_level2                         = 0x00;
uint8_t                 app_bas_battery_level_client_char_config2[]    = {0x00, 0x00, };

// 0x02: second battery (Bluetooth SIG namespace)
uint8_t                 app_bas_battery_level_presentation_format2[]   = {0x04, 0x00, 0x04, 0x2B, 0x01, 0x01, 0x00};
#endif

 /************************************************************************************
 * GATT Lookup Table
 ************************************************************************************/

gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl_1_1[] =
{
    /* { attribute handle,                       maxlen, curlen, attribute data } */
    { HDLC_GAP_DEVICE_NAME_VALUE,                 15,     15,     app_gap_device_name },
    { HDLC_GAP_APPEARANCE_VALUE,                  2,      2,      app_gap_appearance },
    { HDLC_BAS_BATTERY_LEVEL_VALUE,               1,      1,      &app_bas_battery_level[0] },
    { HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG,  2,      2,      app_bas_battery_level_client_char_config },
#ifdef SECOND_BATTERY
    { HDLD_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT, 7,      7,      app_bas_battery_level_presentation_format },
#endif
    /* { attribute handle,                                  maxlen,                         curlen,                         attribute data } */
#ifdef BATTERY_LEVEL_STATUS
    { HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE,                  sizeof(batt_level_status_t),    sizeof(batt_level_status_t),   (unsigned char *)&app_bas_battery_level_status },
    { HDLC_BAS_BATTERY_LEVEL_STATUS_CLIENT_CHAR_CONFIG,     2,                              2,                              app_bas_battery_level_status_client_char_config },

#ifdef BATTERY_LEVEL_BROADCAST
    { HDLC_BAS_BATTERY_LEVEL_STATUS_SERVER_CHAR_CONFIG,     2,                              2,                              app_bas_battery_level_status_server_char_config },
#endif

#endif
#ifdef ESTIMATED_SERVICE_DATE
    { HDLC_BAS_ESTIMATED_SERVICE_DATE_VALUE,  sizeof(batt_estimated_service_date_t), sizeof(batt_estimated_service_date_t), (unsigned char *)&app_bas_estimated_service_date },
    { HDLC_BAS_ESTIMATED_SERVICE_DATE_CLIENT_CHAR_CONFIG,   2,                              2,                              app_bas_estimated_service_date_client_char_config },
#endif
#ifdef BATTERY_CRITICAL_STATUS
    { HDLC_BAS_BATTERY_CRITICAL_STATUS_VALUE,               sizeof(batt_critical_status_t), sizeof(batt_critical_status_t), (unsigned char *)&app_bas_critical_status },
    { HDLC_BAS_BATTERY_CRITICAL_STATUS_CLIENT_CHAR_CONFIG,  2,                              2,                              app_bas_critical_status_client_char_config },
#endif
#ifdef BATTERY_ENERGY_STATUS
    { HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE,                 sizeof(batt_energy_status_t),   sizeof(batt_energy_status_t),   (unsigned char *)&app_bas_energy_status },
    { HDLC_BAS_BATTERY_ENERGY_STATUS_CLIENT_CHAR_CONFIG,    2,                              2,                              app_bas_energy_status_client_char_config },
#endif
#ifdef BATTERY_TIME_STATUS
    { HDLC_BAS_BATTERY_TIME_STATUS_VALUE,                   sizeof(batt_time_status_t),     sizeof(batt_time_status_t),     (unsigned char *)&app_bas_time_status },
    { HDLC_BAS_BATTERY_TIME_STATUS_CLIENT_CHAR_CONFIG,      2,                              2,                              app_bas_time_status_client_char_config },
#endif
#ifdef BATTERY_HEALTH_STATUS
    { HDLC_BAS_BATTERY_HEALTH_STATUS_VALUE,                 sizeof(batt_health_status_t),   sizeof(batt_health_status_t),   (unsigned char *)&app_bas_health_status },
    { HDLC_BAS_BATTERY_HEALTH_STATUS_CLIENT_CHAR_CONFIG,    2,                              2,                              app_bas_health_status_client_char_config },
#endif
#ifdef BATTERY_HEALTH_INFO
    { HDLC_BAS_BATTERY_HEALTH_INFO_VALUE,                   sizeof(batt_health_info_t),     sizeof(batt_health_info_t),     (unsigned char *)&app_bas_health_info },
    { HDLC_BAS_BATTERY_HEALTH_INFO_CLIENT_CHAR_CONFIG,      2,                              2,                              app_bas_health_info_client_char_config },
#endif
#ifdef BATTERY_INFO
    { HDLC_BAS_BATTERY_INFO_VALUE,                          sizeof(batt_info_t),            sizeof(batt_info_t),            (unsigned char *)&app_bas_info },
    { HDLC_BAS_BATTERY_INFO_CLIENT_CHAR_CONFIG,             2,                              2,                              app_bas_info_client_char_config },
#endif
#ifdef BATTERY_MANUFACTURE_NAME
    { HDLC_BAS_MANUFACTURE_NAME_VALUE,                      sizeof(app_bas_manuf_name),     sizeof(app_bas_manuf_name),     app_bas_manuf_name },
    { HDLC_BAS_MANUFACTURE_NAME_CLIENT_CHAR_CONFIG,         2,                              2,                              app_bas_manuf_name_client_char_config },
#endif
#ifdef BATTERY_MANUFACTURE_NUMBER
    { HDLC_BAS_MANUFACTURE_NUMBER_VALUE,                    sizeof(app_bas_manuf_num),      sizeof(app_bas_manuf_num),      app_bas_manuf_num },
    { HDLC_BAS_MANUFACTURE_NUMBER_CLIENT_CHAR_CONFIG,       2,                              2,                              app_bas_manuf_num_client_char_config },
#endif
#ifdef BATTERY_SERIAL_NUMBER
    { HDLC_BAS_SERIAL_NUMBER_VALUE,                         sizeof(app_bas_serial_num),     sizeof(app_bas_serial_num),     app_bas_serial_num },
    { HDLC_BAS_SERIAL_NUMBER_CLIENT_CHAR_CONFIG,            2,                              2,                              app_bas_serial_num_client_char_config },
#endif
#ifdef SECOND_BATTERY
    { HDLC_BAS_BATTERY_LEVEL_VALUE2,               1,      1,      &app_bas_battery_level2 },
    { HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG2,  2,      2,      app_bas_battery_level_client_char_config2 },
    { HDLD_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT2, 7,      7,      app_bas_battery_level_presentation_format2 },
#endif
};

/* Number of Lookup Table entries */
const uint16_t app_gatt_db_ext_attr_tbl_size_1_1 = (sizeof(app_gatt_db_ext_attr_tbl_1_1) / sizeof(gatt_db_lookup_table_t));

#endif // BAS_1_1

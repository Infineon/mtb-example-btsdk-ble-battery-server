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
* File Name: cycfg_gatt_db_1_1.h
*
* Description:
* Definitions for constants used in the device's GATT database and function
* prototypes.
*
********************************************************************************
* Copyright 2021 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
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

#if !defined(CYCFG_GATT_DB_1_1_H)
#define CYCFG_GATT_DB_1_1_H

#define __UUID_CHARACTERISTIC_BATTERY_LEVEL_STATUS              0x7F90
#define __UUID_CHARACTERISTIC_BATTERY_ESTIMATED_SERVICE_DATE    0x7F91
#define __UUID_CHARACTERISTIC_BATTERY_CRITICAL_STATUS           0x7F92
#define __UUID_CHARACTERISTIC_BATTERY_ENERGY_STATUS             0x7F93
#define __UUID_CHARACTERISTIC_BATTERY_TIME_STATUS               0x7F94
#define __UUID_CHARACTERISTIC_BATTERY_HEALTH_STATUS             0x7F95
#define __UUID_CHARACTERISTIC_BATTERY_HEALTH_INFO               0x7F96
#define __UUID_CHARACTERISTIC_BATTERY_INFO                      0x7F97
#define __UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NAME          0x2A29
#define __UUID_CHARACTERISTIC_BATTERY_MANUFACTURE_NUMBER        0x2A24
#define __UUID_CHARACTERISTIC_BATTERY_SERIAL_NUMBER             0x2A25

/* Descriptor Battery level presentation format  */
#define HDLD_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT              0x0020

/* Characteristic Battery Level Status */
#define HDLC_BAS_BATTERY_LEVEL_STATUS                           0x0030
#define HDLC_BAS_BATTERY_LEVEL_STATUS_VALUE                     0x0031
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_BATTERY_LEVEL_STATUS_CLIENT_CHAR_CONFIG        0x0032

#ifdef BATTERY_LEVEL_BROADCAST
/* Descriptor Server Characteristic Configuration */
#define HDLC_BAS_BATTERY_LEVEL_STATUS_SERVER_CHAR_CONFIG        0x0033
#endif

/* Characteristic Battery Estimated Sevice Date */
#define HDLC_BAS_ESTIMATED_SERVICE_DATE                         0x0040
#define HDLC_BAS_ESTIMATED_SERVICE_DATE_VALUE                   0x0041
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_ESTIMATED_SERVICE_DATE_CLIENT_CHAR_CONFIG      0x0042

/* Characteristic Battery Critical Status */
#define HDLC_BAS_BATTERY_CRITICAL_STATUS                        0x0050
#define HDLC_BAS_BATTERY_CRITICAL_STATUS_VALUE                  0x0051
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_BATTERY_CRITICAL_STATUS_CLIENT_CHAR_CONFIG     0x0052

/* Characteristic Battery Energy Status */
#define HDLC_BAS_BATTERY_ENERGY_STATUS                          0x0060
#define HDLC_BAS_BATTERY_ENERGY_STATUS_VALUE                    0x0061
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_BATTERY_ENERGY_STATUS_CLIENT_CHAR_CONFIG       0x0062

/* Characteristic Battery Time Status */
#define HDLC_BAS_BATTERY_TIME_STATUS                            0x0070
#define HDLC_BAS_BATTERY_TIME_STATUS_VALUE                      0x0071
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_BATTERY_TIME_STATUS_CLIENT_CHAR_CONFIG         0x0072

/* Characteristic Battery Health Status */
#define HDLC_BAS_BATTERY_HEALTH_STATUS                          0x0080
#define HDLC_BAS_BATTERY_HEALTH_STATUS_VALUE                    0x0081
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_BATTERY_HEALTH_STATUS_CLIENT_CHAR_CONFIG       0x0082

/* Characteristic Battery Health Information */
#define HDLC_BAS_BATTERY_HEALTH_INFO                            0x0090
#define HDLC_BAS_BATTERY_HEALTH_INFO_VALUE                      0x0091
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_BATTERY_HEALTH_INFO_CLIENT_CHAR_CONFIG         0x0092

/* Characteristic Battery Information */
#define HDLC_BAS_BATTERY_INFO                                   0x00a0
#define HDLC_BAS_BATTERY_INFO_VALUE                             0x00a1
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_BATTERY_INFO_CLIENT_CHAR_CONFIG                0x00a2

/* Characteristic Battery Manufacture Name */
#define HDLC_BAS_MANUFACTURE_NAME                               0x00b0
#define HDLC_BAS_MANUFACTURE_NAME_VALUE                         0x00b1
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_MANUFACTURE_NAME_CLIENT_CHAR_CONFIG            0x00b2

/* Characteristic Battery Manufacture Number */
#define HDLC_BAS_MANUFACTURE_NUMBER                             0x00c0
#define HDLC_BAS_MANUFACTURE_NUMBER_VALUE                       0x00c1
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_MANUFACTURE_NUMBER_CLIENT_CHAR_CONFIG          0x00c2

/* Characteristic Battery Serial Number */
#define HDLC_BAS_SERIAL_NUMBER                                  0x00d0
#define HDLC_BAS_SERIAL_NUMBER_VALUE                            0x00d1
/* Descriptor Client Characteristic Configuration */
#define HDLC_BAS_SERIAL_NUMBER_CLIENT_CHAR_CONFIG               0x00d2

/* Service Battery2 */
#define HDLS_BAS2                                               0x00e1
/* Characteristic Battery2 Level */
#define HDLC_BAS_BATTERY_LEVEL2                                 0x00e2
#define HDLC_BAS_BATTERY_LEVEL_VALUE2                           0x00e3
/* Descriptor Client Characteristic Configuration */
#define HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG2              0x00e4
/* Descriptor Battery2 level presentation format  */
#define HDLD_BAS_BATTERY_LEVEL_PRESENTATION_FORMAT2             0x00e5

extern const uint8_t gatt_database_1_1[];
extern const uint16_t gatt_database_len_1_1;

extern gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl_1_1[];
extern const uint16_t app_gatt_db_ext_attr_tbl_size_1_1;

#endif /* CYCFG_GATT_DB_1_1_H */

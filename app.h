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
 * Remote control
 *
 * This file provides definitions and function prototypes for remote control
 * device
 *
 */

#ifndef __APP_H__
#define __APP_H__
#include "wiced.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "hidd_lib.h"
#include "btstack.h"

/*******************************************************************************
* Types and Defines
*******************************************************************************/
#if defined(BLE_SUPPORT) && defined(BR_EDR_SUPPORT)
 #define BT_LOCAL_NAME "IFX DUAL MOUSE"
#elif defined(BLE_SUPPORT)
 #define BT_LOCAL_NAME "IFX LE MOUSE"
#else
 #define BT_LOCAL_NAME "IFX CLASSIC MOUSE"
#endif

#if is_20835Family
 #define NUM_KEYSCAN_ROWS    5  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    4  // Num of Cols in keyscan matrix
 #define CONNECT_KEY_INDEX   18 // need to find out from hardware
#else
 #define NUM_KEYSCAN_ROWS    7  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    7  // Num of Cols in keyscan matrix
 #define CONNECT_KEY_INDEX   29 // need to find out from hardware
#endif

#define LED_RED_INDEX        0
#define LED_GREEN_INDEX      1

#define LINK_LED             LED_GREEN_INDEX
#define RED_LED              LED_RED_INDEX

/*******************************************************************************
 * Report ID defines
 ********************************************************************************/
// Input report id
typedef enum {
    RPT_ID_IN_STD_KEY      =0x01,
    RPT_ID_IN_BATTERY      =0x03,
    RPT_ID_IN_MOUSE        =0x08,
    RPT_ID_IN_MOUSE_BOOT   =0x09,
    RPT_ID_IN_CNT_CTL      =0xcc,
    RPT_ID_IN_NOT_USED     =0xff,
    RPT_ID_CLIENT_CHAR_CONF=0xff,
} rpt_id_in_e;

/*******************************************************************************
 * App queue defines
 ********************************************************************************/
typedef union {
    uint8_t                     type;
    HidEvent                    info;
    HidEventMotionXY            mouse;
    HidEventButtonStateChange   button;
    HidEventAny                 any;
    HidEventUserDefine          user;
} app_queue_t;

#define APP_QUEUE_SIZE sizeof(app_queue_t)
#define APP_QUEUE_MAX  44                         // max number of event in queue

#define MOUSE_XY_DATA_SIZE 12                     // Mouse x,y data size is 12-bit, must be either 8, 12, or 16

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Include all components
 *******************************************************************************/
#include "battery.h"
#include "ota.h"
#include "bt.h"
#include "mouse.h"
#include "key.h"

typedef struct {
    uint8_t protocol;
    wiced_hidd_app_event_queue_t eventQueue;
    app_queue_t events[APP_QUEUE_MAX];
    uint8_t pollSeqn;
    uint8_t recoveryInProgress;
    uint8_t setReport_status;
    uint8_t firstTransportStateChangeNotification:1;
    uint8_t enter_pairing_pending:1;
} app_t;

extern app_t app;

/********************************************************************************
 * Function Name: app_setProtocol
 ********************************************************************************
 * Summary:
 *  This function implements the rxSetReport function defined by
 *  the HID application to handle "Set Report" messages.
 *  This function looks at the report ID and passes the message to the
 *  appropriate handler.
 *
 * Parameters:
 *  reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *  reportId -- report id
 *  payload -- pointer to data that came along with the set report request after the report ID
 *  payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_setProtocol(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize);

/*******************************************************************************
 * Function Name: app_queueEvent
 ********************************************************************************
 * Summary:
 *  Queue an event to event queue
 *
 * Parameters:
 *  event -- event to queue
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_queueEvent(app_queue_t * event);

/*******************************************************************************
 * Function Name: app_transportStateChangeNotification
 ********************************************************************************
 * Summary:
 *  This function is called when the state of a link is changed.
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_transportStateChangeNotification(uint32_t newState);

/*******************************************************************************
 * Function Name: app_enter_pairing
 ********************************************************************************
 * Summary: enter pairing
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_enter_pairing(void);

/*******************************************************************************
 * Function Name: app_start
 ********************************************************************************
 * Summary: This is application start function. After system initialization is done, when the
 *          Bluetooth management calls with BTM_ENABLED_EVT, this function is called to
 *          start application
 *
 * Parameters:
 *  none
 *
 * Return:
 *  WICED_BT_SUCCESS -- if application initialization is okay and ready to start;
 *                      otherwise, should return the error code defined in wiced_result_t
 *
 *******************************************************************************/
wiced_result_t app_start();

#endif // __BLEREMOTE_H__

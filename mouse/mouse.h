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
 * Keyscan Interface definitions
 *
 */
#ifndef __MOUSE_H__
#define __MOUSE_H__

#include "wiced.h"

#pragma pack(1)
/// Boot mode mouse report
typedef struct
{
    /// HID report ID
    uint8_t    reportID;
    /// Button state in bitmap
    uint8_t    buttonState;
    /// The accumulcatd X motion of sensor
    int8_t    xMotion;
    /// The accumulcatd Y motion of sensor
    int8_t    yMotion;
    /// The scroll wheel motion
    int8_t    scroll;
}BootModeReport;

/// Report mode mouse report with 12 bit X/Y
typedef struct
{
    /// HID report ID
    uint8_t    reportID;
    /// Button state in bitmap
    uint8_t    buttonState;

#if (MOUSE_XY_DATA_SIZE==16)
    /// X motion lower 16 bits
    int16_t    xMotion;
    /// Y motion upper 16 bits
    int16_t    yMotion;
    /// The scroll wheel motion
    int8_t     scroll;
#elif (MOUSE_XY_DATA_SIZE==12)
    /// X motion lower 8 bits
    int32_t    xMotion:12;

    int32_t    yMotion:12;

    /// Y motion upper 8 bits
    int32_t    scroll:8;
#else
    /// X motion lower 8 bits
    uint8_t    xMotion;
    /// Y motion upper 8 bits
    uint8_t    yMotion;
    /// The scroll wheel motion
    int8_t     scroll;
#endif

}ReportModeReport;
#pragma pack()

#ifdef MOUSE_REPORT_SUPPORT
 #ifdef USE_LSM9DS1
   #include "lsm9ds1.h"
 #endif
 #include "hidd_lib.h"
 #include "scroll.h"
 #include "motion.h"
 #include "button.h"

typedef struct
{
    /// Boot mode motion report
    BootModeReport bootModeReport;

    /// Report mode motion report
    ReportModeReport reportModeReport;
} mouse_t;

extern mouse_t mouse;

/*******************************************************************************
 * Function Name: mouse_button_xy_changed(int x, int y)
 ********************************************************************************
 * Summary: mouse_button_xy_changed
 *
 * Parameters:
 *  int x,y   -- x,y delta
 *  int s     -- scroll
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void mouse_button_xys_changed(int x, int y, int s);

/*******************************************************************************
 * Function Name: mouse_button_state_changed(uint8_t newState)
 ********************************************************************************
 * Summary: mouse_button_state_changed
 *
 * Parameters:
 *  uint8_t newState ; new button state
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void mouse_button_state_changed(uint8_t newState);

/*******************************************************************************
 * Function Name: mouse_init()
 ********************************************************************************
 * Summary: Initialize mouse report support
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
 void mouse_init(void);

////////////////////////////////////////////////////////////////////////////////
/// Function Name: mouse_poll
////////////////////////////////////////////////////////////////////////////////
/// Polls mouse data and send if there is new data.
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
 void mouse_poll(void);

////////////////////////////////////////////////////////////////////////////////
/// Function Name: mouse_shutdown
////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
 void mouse_shutdown(void);

#else
 #define mouse_poll()
 #define mouse_init()
 #define mouse_shutdown()
#endif // MOUSE_REPORT_SUPPORT

#endif

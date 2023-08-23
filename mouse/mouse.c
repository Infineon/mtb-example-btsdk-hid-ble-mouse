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
 * mouse.c
 *
 */
#include "app.h"

#ifdef MOUSE_REPORT_SUPPORT

mouse_t mouse = {
.bootModeReport.reportID = RPT_ID_IN_MOUSE_BOOT,
.reportModeReport.reportID = RPT_ID_IN_MOUSE,
};

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
void mouse_button_xys_changed(int x, int y, int s)
{
    if (x || y || s) // any delta
    {
        if (app.protocol==PROTOCOL_REPORT)
        {
            mouse.reportModeReport.xMotion = x;
            mouse.reportModeReport.yMotion = y;
            mouse.reportModeReport.scroll = s;
            hidd_link_send_report(&mouse.reportModeReport, sizeof(ReportModeReport));
        }
        else
        {
            mouse.bootModeReport.xMotion = x;
            mouse.bootModeReport.yMotion = y;
            mouse.bootModeReport.scroll = s;
            hidd_link_send_report(&mouse.bootModeReport, sizeof(BootModeReport));
        }
    }
}

/*******************************************************************************
 * Function Name: mouse_button_state_changed(uint16_t newState)
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
void mouse_button_state_changed(uint8_t newState)
{
    if (app.protocol==PROTOCOL_REPORT)
    {
        memset(&mouse.reportModeReport.buttonState,0,sizeof(ReportModeReport)-1);
        mouse.reportModeReport.buttonState = newState;
        hidd_link_send_report(&mouse.reportModeReport, sizeof(ReportModeReport));
    }
    else
    {
        memset(&mouse.bootModeReport.buttonState,0,sizeof(BootModeReport)-1);
        mouse.bootModeReport.buttonState = newState;
        hidd_link_send_report(&mouse.bootModeReport, sizeof(BootModeReport));
    }
}

/*******************************************************************************
 * Function Name: mouse_poll()
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
void mouse_poll(void)
{
    scroll_poll();
    motion_poll();
    button_poll();
}

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
void mouse_init(void)
{
    scroll_init();
    motion_init();
    button_init();
}

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
void mouse_shutdown(void)
{
    scroll_shutdown();
    motion_shutdown();
    button_shutdown();
}

#endif // MOUSE_REPORT_SUPPORT

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
 * scroll.c
 *
 */
#include "scroll.h"
#ifdef SCROLL_SUPPORT

void scroll_poll()
{
#if 0
    int16_t scrollCurrent;

#if 1
    scrollCurrent = wiced_hal_quadrature_get_scroll_count();
#else
    int16_t scroll;
    int16_t readCount = blemouseAppConfig.maxNumXYReadsPerPoll * 2;

    scrollCurrent = 0;
    while ((scroll = wiced_hal_quadrature_get_scroll_count()) && readCount--)
        scrollCurrent += scroll;
#endif

    // Check for scroll
    if (scrollCurrent)
    {
        //WICED_BT_TRACE("\nsc:%d", scrollCurrent);
        // Negate scroll value if enabled
        if (blemouseAppConfig.negateScroll)
        {
            scrollCurrent = -scrollCurrent;
        }

        // Check if scroll scaling is enabled
        if (blemouseAppConfig.scrollScale)
        {
            // Yes. Add the current scroll count to the fractional count
            mouseAppState->mouseapp_scrollFractional += scrollCurrent;

            // Scale and adjust accumulated scroll value. Fractional value will be
            // left in the factional part. Place the whole number in the scroll
            // event
            mouseAppState->mouseapp_scrollEvent.motion =
                mouseapp_scaleValue(&mouseAppState->mouseapp_scrollFractional, blemouseAppConfig.scrollScale);

            // Reset the scroll discard counter
            mouseAppState->mouseapp_pollsSinceScroll = 0;
        }
        else
        {
            // No scaling is required. Put the data in the scroll event
            mouseAppState->mouseapp_scrollEvent.motion = scrollCurrent;
        }

        // Queue scroll event with the proper seqn
        wiced_hidd_event_queue_add_event_with_overflow(&mouseAppState->mouseappEventQueue,
                                          &mouseAppState->mouseapp_scrollEvent.eventInfo, sizeof(mouseAppState->mouseapp_scrollEvent), mouseAppState->mouseapp_pollSeqn);
    }
    else
    {
        // If scroll scaling timeout is not infinite, bump up the
        // inactivity counter and check if we have crossed the threshold.
        if (blemouseAppConfig.pollsToKeepFracScrollData &&
            ++mouseAppState->mouseapp_pollsSinceScroll >= blemouseAppConfig.pollsToKeepFracScrollData)
        {
            // We have. Discard any fractional scroll data
            mouseAppState->mouseapp_scrollFractional = 0;

            // Reset the scroll discard counter
            mouseAppState->mouseapp_pollsSinceScroll = 0;
        }
    }
#endif
}

void scroll_shutdown()
{
    // Disable the quadrature HW
    wiced_hal_quadrature_turnOff();
}

void scroll_init()
{
    #define USE_P26_QOC TRUE,FALSE,FALSE,FALSE
    #define USE_P27_QOC FALSE,TRUE,FALSE,FALSE
    #define USE_P28_QOC FALSE,FALSE,TRUE,FALSE
    #define USE_P29_QOC FALSE,FALSE,FALSE,TRUE
	#define USE_X_AXIS (CH_Z_DISABLE|CH_XY_ENABLE|CH_XY_SEL_LHL_PWM_RATE),WICED_TRUE,WICED_FALSE,WICED_FALSE
	#define USE_Y_AXIS (CH_Z_DISABLE|CH_XY_ENABLE|CH_XY_SEL_LHL_PWM_RATE),WICED_FALSE,WICED_TRUE,WICED_FALSE
    #define USE_Z_AXIS (CH_Z_ENABLE|CH_XY_DISABLE|CH_Z_SAMPLE_ONCE_PER_LHL_PWM),WICED_FALSE,WICED_FALSE,WICED_TRUE

    wiced_hal_quadrature_configure(QUADRATURE_LED_CONFIG_SOURCE |            //QOC_LEDs_output_polarity for QOC0 bit[1:0]
                                   QUADRATURE_LED_CONFIG_SOURCE << 2 |       //QOC_LEDs_output_polarity for QOC1 bit[3:2]
                                   QUADRATURE_LED_CONFIG_SOURCE << 4 |       //QOC_LEDs_output_polarity for QOC2 bit[5:4]
                                   QUADRATURE_LED_CONFIG_SOURCE << 6,        //QOC_LEDs_output_polarity for QOC3 bit[7:6]
                                   GPIO_PULL_DOWN | GPIO_EN_INT_RISING_EDGE, //quadratureInputGpioConfig
                                   ENABLE_PORT_0_PINS_AS_QUAD_INPUT,         //port0PinsUsedAsQuadratureInput (Use Port0 set)
                                                                             //  Port0 set = P2-P7, (P2 as qdx0, P3 as qdx1, P4 as qdy0, P5 as qdy1, P6 as qdz0,P7 as qdz1)
                                                                             //  Port2 set = P32-P37, (P32 as qdx0, P33 as qdx1, P34 as qdy0, P35 as qdy1, P36 as qdz0, P37 as qdz1)
  #if defined(CYW20819A1)
    // 20819 configure P28 for QOC, Y-Axis (P4,P5) based on HW schematics in referenced design.
                                   USE_P28_QOC,
                                   USE_Y_AXIS);
  #else
    // 20735 configure P29 for QOC, Z-Axis (P6,P7) based on HW schematics in referenced design.
                                   USE_P29_QOC,
                                   USE_Z_AXIS);
  #endif
    //quadrature driver init
    wiced_hal_quadrature_init();
}

#endif //SCROLL_SUPPORT

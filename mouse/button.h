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
 * scroll.h
 * Mouse scroll is implemented with quadrature
 *
 */
#ifndef __BUTTON_H__
#define __BUTTON_H__

#ifdef CLICK_SUPPORT

enum {
    NO_BUTTON_BIT    = 0x0000,
    LEFT_BUTTON_BIT  = 0x0001,
    RIGHT_BUTTON_BIT = 0x0002,
    MID_BUTTON_BIT   = 0x0004,
    PAIR_BUTTON_BIT  = 0x8000,
};
 #define CONNECT_COMBO_HOLD_TIME 10                  // 10 sec to enter pairing mode

 #ifdef MOUSE_PLATFORM
  #define CONNECT_COMBO  (LEFT_BUTTON_BIT | RIGHT_BUTTON_BIT)
  #define CLICK_USE_KEYSCAN
  #define BUTTON_COUNT 3
 #else
  #define CONNECT_COMBO  LEFT_BUTTON_BIT
  #define BUTTON_COUNT 1
 #endif

 void button_init();
 void button_poll();
 #ifdef CLICK_USE_KEYSCAN
  #dfeine button_shutdown() wiced_hal_keyscan_turnOff()
 #else
  #define button_shutdown()
 #endif
#else
 #define button_init()
 #define button_shutdown()
 #define button_poll()
#endif  // CLICK_SUPPORT

#endif  // __BUTTON_H__

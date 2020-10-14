/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
 * BR/EDR function and data
 *
 */
#ifndef __APP_BT_H__
#define __APP_BT_H__

#include "ble.h"
#include "bredr.h"

extern wiced_bt_cfg_settings_t bt_cfg;
extern uint8_t dev_local_name[];
extern uint8_t blehid_rpt_map[];

#ifdef KEY_REPORT_SUPPORT
#define STD_KB_REPORT_DESCRIPTOR \
    /* RPT_ID_IN_STD_KEY */ \
    /* Input Report, 8 bytes */ \
    /* 1st byte:Keyboard LeftControl/Keyboard Right GUI */ \
    /* 2nd byte:Constant, 3rd ~ 6th: keycode */ \
    /* Output Report, 1 byte: LED control */ \
    0x05, 0x01,                    /* USAGE_PAGE (Generic Desktop) */ \
    0x09, 0x06,                    /* USAGE (Keyboard) */ \
    0xA1, 0x01,                    /* COLLECTION (Application) */ \
    0x85, RPT_ID_IN_STD_KEY,       /*    REPORT_ID */ \
    0x75, 0x01,                    /*    REPORT_SIZE (1) */ \
    0x95, 0x08,                    /*    REPORT_COUNT (8) */ \
    0x05, 0x07,                    /*    USAGE_PAGE (Keyboard) */ \
    0x19, 0xE0,                    /*    USAGE_MINIMUM (Keyboard LeftControl) */ \
    0x29, 0xE7,                    /*    USAGE_MAXIMUM (Keyboard Right GUI) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x25, 0x01,                    /*    LOGICAL_MAXIMUM (1) */ \
    0x81, 0x02,                    /*    INPUT (Data,Var,Abs) */ \
    0x95, 0x01,                    /*    REPORT_COUNT (1) */ \
    0x75, 0x08,                    /*    REPORT_SIZE (8) */ \
    0x81, 0x03,                    /*    INPUT (Cnst,Var,Abs) */ \
    0x95, 0x05,                    /*    REPORT_COUNT (5) */ \
    0x75, 0x01,                    /*    REPORT_SIZE (1) */ \
    0x05, 0x08,                    /*    USAGE_PAGE (LEDs) */ \
    0x19, 0x01,                    /*    USAGE_MINIMUM (Num Lock) */ \
    0x29, 0x05,                    /*    USAGE_MAXIMUM (Kana) */ \
    0x91, 0x02,                    /*    OUTPUT (Data,Var,Abs) */ \
    0x95, 0x01,                    /*    REPORT_COUNT (1) */ \
    0x75, 0x03,                    /*    REPORT_SIZE (3) */ \
    0x91, 0x03,                    /*    OUTPUT (Cnst,Var,Abs) */ \
    0x95, 0x06,                    /*    REPORT_COUNT (6) */ \
    0x75, 0x08,                    /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF, 0x00,              /*    LOGICAL_MAXIMUM (255) */ \
    0x05, 0x07,                    /*    USAGE_PAGE (Keyboard) */ \
    0x19, 0x00,                    /*    USAGE_MINIMUM (Reserved (no event indicated)) */ \
    0x29, 0xFF,                    /*    USAGE_MAXIMUM (Reserved (no event indicated)) */ \
    0x81, 0x00,                    /*    INPUT (Data,Ary,Abs) */ \
    0xC0,                          /* END_COLLECTION */
#else
#define STD_KB_REPORT_DESCRIPTOR
#endif

#define MOUSE_BOOT_REPORT_DESCRIPTOR \
    /* 8-bit Mouse report, RPT_ID_IN_MOUSE_BOOT */ \
    0x05, 0x01,             /* Usage Page (Generic Desktop), */ \
    0x09, 0x02,             /* Usage (Mouse), */ \
    0xA1, 0x01,             /* Collection: (Application), */ \
    0x85, RPT_ID_IN_MOUSE_BOOT,  /*     REPORT_ID  */ \
    0x09, 0x01,             /*     Usage (Pointer), */ \
    0xA1, 0x00,             /*     Collection: (Linked), */ \
    /* byte 0  bit 0(LEFT), 1(RIGHT), 2: button(MID). bit 3-7 not used. */ \
    0x05, 0x09,             /*         Usage Page (Buttons), */ \
    0x19, 0x01,             /*         Usage Minimum (01), */ \
    0x29, 0x03,             /*         Usage Maximum (03), */ \
    0x15, 0x00,             /*         Log Min (0), */ \
    0x25, 0x01,             /*         Log Max (1), */ \
    0x75, 0x01,             /*         Report Size (1), */ \
    0x95, 0x03,             /*         Report Count (03), */ \
    0x81, 0x02,             /*         Input (Data, Variable, Absolute), */ \
    0x75, 0x05,             /*         Report Size (5), */ \
    0x95, 0x01,             /*         Report Count (1), */ \
    0x81, 0x01,             /*         Input (Constant), */ \
    0x05, 0x01,             /*         Usage Page (Generic Desktop), */ \
    0x09, 0x30,             /*         Usage (X), */ \
    0x09, 0x31,             /*         Usage (Y), */ \
    0x15, 0x81,             /*         Logical min (-127), */ \
    0x25, 0x7F,             /*         Logical Max (127), */ \
    0x75, 8,                /*         Report Size (8), */ \
    0x95, 0x02,             /*         Report Count (2) (X,Y) */ \
    0x81, 0x06,             /*         Input (Data, Variable, Relative), */ \
    /* wheel: 8bits */ \
    0x09, 0x38,             /*         Usage (Wheel), */ \
    0x15, 0x81,             /*         Logical min (-127), */ \
    0x25, 0x7F,             /*         Logical Max (127), */ \
    0x75, 0x08,             /*         Report Size (8), */ \
    0x95, 0x01,             /*         Report Count (1) (Wheel) */ \
    0x81, 0x06,             /*         Input (Data, Variable, Relative), */ \
    0xC0,                   /*     END_COLLECTION (Logical) */ \
    0xC0,                   /* END_COLLECTION */

#ifdef MOUSE_REPORT_SUPPORT

 #if (MOUSE_XY_DATA_SIZE==16)
    /* x: 16 bits, y:16 bits.  4 bytes */
  #define MOUSE_SIZE_DESCRIPTOR \
    0x16, 0x01, 0x80,       /*          Logical min (-32,767), */ \
    0x26, 0xFF, 0x7F,       /*          Logical Max (32,767), */ \
    0x75, 16,               /*          Report Size (16), */
 #elif (MOUSE_XY_DATA_SIZE==12)
    /* x: 12 bits, y:12 bits.  3 bytes */
  #define MOUSE_SIZE_DESCRIPTOR \
    0x16, 0x01, 0xF8,       /*          Logical min (-2047), */ \
    0x26, 0xFF, 0x07,       /*          Logical Max (2047), */ \
    0x75, 12,               /*          Report Size (12), */
 #else
    /* x: 8 bits, y:8 bits.  2 bytes */
  #define MOUSE_SIZE_DESCRIPTOR \
    0x15, 0x81,             /*         Logical min (-127), */ \
    0x25, 0x7F,             /*         Logical Max (127), */ \
    0x75, 0x08,             /*         Report Size (8), */
 #endif

 #define MOUSE_REPORT_DESCRIPTOR \
    /* 16-bit Mouse report, RPT_ID_IN_MOUSE */ \
    0x05, 0x01,             /* Usage Page (Generic Desktop), */ \
    0x09, 0x02,             /* Usage (Mouse), */ \
    0xA1, 0x01,             /* Collection: (Application), */ \
    0x85, RPT_ID_IN_MOUSE,  /*     REPORT_ID  */ \
    0x09, 0x01,             /*     Usage (Pointer), */ \
    0xA1, 0x00,             /*     Collection: (Linked), */ \
    /* byte 0  bit 0(LEFT), 1(RIGHT), 2: button(MID). bit 3-7 not used. */ \
    0x05, 0x09,             /*         Usage Page (Buttons), */ \
    0x19, 0x01,             /*         Usage Minimum (01), */ \
    0x29, 0x03,             /*         Usage Maximum (03), */ \
    0x15, 0x00,             /*         Log Min (0), */ \
    0x25, 0x01,             /*         Log Max (1), */ \
    0x75, 0x01,             /*         Report Size (1), */ \
    0x95, 0x03,             /*         Report Count (03), */ \
    0x81, 0x02,             /*         Input (Data, Variable, Absolute), */ \
    0x75, 0x05,             /*         Report Size (5), */ \
    0x95, 0x01,             /*         Report Count (1), */ \
    0x81, 0x01,             /*         Input (Constant), */ \
    0x05, 0x01,             /*         Usage Page (Generic Desktop), */ \
    0x09, 0x30,             /*         Usage (X), */ \
    0x09, 0x31,             /*         Usage (Y), */ \
    MOUSE_SIZE_DESCRIPTOR \
    0x95, 0x02,             /*         Report Count (2) (X,Y) */ \
    0x81, 0x06,             /*         Input (Data, Variable, Relative), */ \
    /* wheel: 8bits */ \
    0x09, 0x38,             /*         Usage (Wheel), */ \
    0x15, 0x81,             /*         Logical min (-127), */ \
    0x25, 0x7F,             /*         Logical Max (127), */ \
    0x75, 0x08,             /*         Report Size (8), */ \
    0x95, 0x01,             /*         Report Count (1) (Wheel) */ \
    0x81, 0x06,             /*         Input (Data, Variable, Relative), */ \
    0xC0,                   /*     END_COLLECTION (Logical) */ \
    0xC0,                   /* END_COLLECTION */

#else
 #define MOUSE_REPORT_DESCRIPTOR
#endif //MOUSE_REPORT_SUPPORT

#ifdef BATTERY_REPORT_SUPPORT
// Use BATTERY_REPORT_DESCRIPTOR for the last entry because it has no ',' in the end
#define BATTERY_REPORT_DESCRIPTOR \
    /*Battery report */ \
    0x05, 0x0C,                    /* Usage Page (Consumer Devices), */ \
    0x09, 0x01,                    /* Usage (Consumer Control), */ \
    0xA1, 0x01,                    /* COLLECTION (Application) */ \
    0x85, RPT_ID_IN_BATTERY,       /*    REPORT_ID (3) */ \
    0x05, 0x06,                    /*      USAGE PAGE (Generic Device Control), */ \
    0x09, 0x20,                    /*      USAGE (Battery Strength), */ \
    0x15, 0x00,                    /*      Log Min (0), */ \
    0x26, 0x64, 0x00,              /*      Log Max (100), */ \
    0x75, 0x08,                    /*      Report Size (8), */ \
    0x95, 0x01,                    /*      Report Count (1), */ \
    0x81, 0x02,                    /*      Input (Data, Variable, Absolute), */ \
    0xC0,                          /*      END_COLLECTION */
#else
#define BATTERY_REPORT_DESCRIPTOR
#endif

#define USB_LE_RPT_DESCRIPTOR \
{\
  STD_KB_REPORT_DESCRIPTOR \
  MOUSE_BOOT_REPORT_DESCRIPTOR \
  MOUSE_REPORT_DESCRIPTOR \
  BATTERY_REPORT_DESCRIPTOR \
}

#define USB_BREDR_RPT_DESCRIPTOR \
  STD_KB_REPORT_DESCRIPTOR \
  MOUSE_BOOT_REPORT_DESCRIPTOR \
  MOUSE_REPORT_DESCRIPTOR \
  BATTERY_REPORT_DESCRIPTOR

/*******************************************************************************
 * Function Name: void bt_init()
 ********************************************************************************
 * Summary: Bluetooth transport init.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void bt_init();

#else
#define bt_init()
#endif // __APP_BT_H__

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
 * button.c
 *
 */
#include "app.h"

#ifdef CLICK_SUPPORT

typedef struct {
#ifdef USE_COMBO_PAIRING
    wiced_timer_t combokey_timer;
#endif
    uint16_t state;
} button_t;

button_t button = {};

#ifdef USE_COMBO_PAIRING
/////////////////////////////////////////////////////////////////////////////////
/// This is a callback function from combo key time out
/////////////////////////////////////////////////////////////////////////////////
static void BUTTON_combo_key_timeout( uint32_t  )
{
    switch (button.state) {
    case CONNECT_COMBO:
        button.state = 0;
        mouse_button_state_changed((uint8_t) button.state);
        hidd_led_off(LED_RED);
        app_enter_pairing();
        break;
    }
}
#endif

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
static void BUTTON_state_changed()
{
#ifdef USE_COMBO_PAIRING
    switch (button.state) {
    case CONNECT_COMBO:
        hidd_led_on(LED_RED);
        WICED_BT_TRACE("\nStart Connect timer");
        wiced_start_timer(&button.combokey_timer, CONNECT_COMBO_HOLD_TIME);
        break;
    default:
        hidd_led_off(LED_RED);
        if (wiced_is_timer_in_use(&button.combokey_timer))
        {
            WICED_BT_TRACE("\nStop connect timer");
            wiced_stop_timer(&button.combokey_timer);
        }
        break;
    }
#endif // USE_COMBO_PAIRING

    mouse_button_state_changed((uint8_t) button.state);
}

#ifdef CLICK_USE_BUTTON
/* This is the pairing button interrupt handler */
static void BUTTON_interrupt_handler( void* user_data, uint8_t pin )
{
    uint32_t mask = (uint32_t) user_data;
    if (wiced_hal_gpio_get_pin_input_status(pin)) // pin pulled high, button press shorting to ground. Thus 1:UP, 0:Down
    {
        button.state &= ~mask;     // up
    }
    else     // button down
    {
        button.state |= mask;     // down
    }

    BUTTON_state_changed();
}

#endif


void button_poll()
{
#ifdef CLICK_USE_KEYSCAN
    uint16_t newState = (uint16_t) wiced_hal_keyscan_button_get_current_state();
    if (newState != button.state)
    {
        button.state = newState;
        BUTTON_state_changed();
    }
#endif
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
void button_init(void)
{
   // Three buttons: button click: p0+P8:LEFT, p0+P9:MID, p0+P10:RIGHT (no pairing keyscan button in demo hardware)
    uint16_t button_report_bits[] = {LEFT_BUTTON_BIT,   //button 'LEFT'. report bitmap 0x0001
                                     MID_BUTTON_BIT,    //button 'MID".report bitmap 0x0004
                                     RIGHT_BUTTON_BIT,  //button 'RIGHT'. report bitmap 0x0002
                                     PAIR_BUTTON_BIT};  //button 'PAIRING'. map it to the last button bit.
    #define BUTTON_CNT (sizeof(button_report_bits)/sizeof(uint16_t))

#ifdef CLICK_USE_KEYSCAN
    wiced_hal_keyscan_button_configure(BUTTON_CNT, WICED_FALSE, WICED_FALSE, button_report_bits);
    //button driver init
    wiced_hal_keyscan_button_init();
#endif
#ifdef CLICK_USE_BUTTON
    extern const size_t button_count;
    int count = button_count > BUTTON_COUNT ? BUTTON_COUNT : button_count; // min(BUTTON_COUNT, button_count)

    while (count--)
    {
        wiced_platform_register_button_callback(count, BUTTON_interrupt_handler, (void *) (uint32_t) button_report_bits[count], WICED_PLATFORM_BUTTON_BOTH_EDGE);
    }

#endif

#ifdef USE_COMBO_PAIRING
    // init combo key
    wiced_init_timer( &button.combokey_timer, BUTTON_combo_key_timeout, 0, WICED_SECONDS_TIMER );
#else // check for Power up click
    extern const wiced_platform_button_config_t platform_button[];
    if (!wiced_hal_gpio_get_pin_input_status(*platform_button[WICED_PLATFORM_BUTTON_1].gpio))
    {
        WICED_BT_TRACE("\nPower up click detected");
        app.enter_pairing_pending = 1;
    }

#endif // USE_COMBO_PAIRING
}

#endif // CLICK_SUPPORT

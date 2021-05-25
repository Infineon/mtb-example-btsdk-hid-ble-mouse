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
* BLE Mouse
*
* The BLE Mouse application is a single chip SoC compliant with HID over GATT Profile (HOGP).
*
* During initialization the app registers with LE stack, WICED HID Device Library and
* keyscan HW to receive various notifications including bonding complete, connection
* status change, peer GATT request/commands and interrupts for button pressed/released.
* Press any button will start LE advertising. When device is successfully bonded, the app
* saves bonded host's information in the NVRAM.
* When user presses/releases button, a HID report will be sent to the host.
* On connection up or battery level changed, a battery report will be sent to the host.
* When battery level is bellowed shutdown voltage, device will critical shutdown.
*
* Features demonstrated
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - Sending HID reports to the host
*  - Processing write requests from the host
*  - Low power management
*  - Over the air firmware update (OTAFWU)
*
* See the readme for instructions.
*/
#include "app.h"
#include "wiced_hal_mia.h"
#include "wiced_memory.h"

#define RECOVERY_COUNT 3

app_t app = {
.protocol = PROTOCOL_REPORT,
.firstTransportStateChangeNotification = 1,
};

/////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_connect_button
/////////////////////////////////////////////////////////////////////////////////
/// Checks for if the current key is connet button.
/// If it is, handle the action accordingly and return TRUE to indicate it is taken care of.
/// Otherwise, it returns FALSE.
///
/// Parameter:
///   keyCode -- the key to check.
///   Down -- TRUE to indicate the key is pressed down.
///
/// Return:
///   TRUE -- keyCode is connect button and it is handled.
///   FALSE -- keyCode is not connect button.
///
////////////////////////////////////////////////////////////////////////////////
static wiced_bool_t APP_connect_button(uint8_t keyCode, wiced_bool_t down)
{
    if (keyCode == CONNECT_KEY_INDEX)
    {
        WICED_BT_TRACE("\nConnect button=%s", down?"DN":"UP");
        if (down)
        {
            app_enter_pairing();
        }
        return TRUE;
    }
    return FALSE;
}

/********************************************************************************
 * Function Name: APP_getProtocol
 ********************************************************************************
 * Summary:
 *   Process get current protocol request. Sends a data transaction over
 *   the control channel of the given transport with the current protocol.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   The current protocol
 *
 *******************************************************************************/
static uint8_t APP_getProtocol(void)
{
    return app.protocol;
}

/********************************************************************************
 * Function Name: APP_setProtocol
 ********************************************************************************
 * Summary:
 *  Handles set protocol from the host. Uses the default hid application function
 *  for setting the protocol. In addition, if the protocol changes and the new protocol
 *  is report, it:
 *      - clears the bit mapped report
 *      - clears the sleep report
 *      - sets the func-lock key as up regardless of its current state
 *
 * Parameters:
 *  newProtocol -- protocol
 *
 * Return:
 *  HID_PAR_HANDSHAKE_RSP_SUCCESS
 *
 *******************************************************************************/
uint8_t APP_setProtocol(uint8_t newProtocol)
{
    ble_setProtocol(newProtocol);   // for BLE table selection
    app.protocol = newProtocol;
    return HID_PAR_HANDSHAKE_RSP_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_procEvtUserDefined
////////////////////////////////////////////////////////////////////////////////
/// Process a user defined event. By default the keyboard application
/// define key and scroll events. If an application needs additional types of
/// events it should define them and override this function to process them.
/// This function should remove the user defined event from the event queue
/// after processing it. This function can consume additional events after
/// the user defined event.
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
static void APP_procEvtUserDefined(void)
{
    if (wiced_hidd_event_queue_get_current_element(&app.eventQueue)!= NULL)
    {
        wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_shutdown
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
static void APP_shutdown(void)
{
    WICED_BT_TRACE("\napp_shutdown");

    // Flush the event queue
    wiced_hidd_event_queue_flush(&app.eventQueue);

    mouse_shutdown();

    if(hidd_link_is_connected())
    {
        hidd_link_disconnect();
    }
    // Disable Interrupts
    wiced_hal_mia_enable_mia_interrupt(FALSE);
    wiced_hal_mia_enable_lhl_interrupt(FALSE);
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_pollActivityUser
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///  This function provides an implementation for the HID application abstract
///  function pollActivityUser(). It polls the following sources for user activity:
///        - Keys
///  Any detected activity is queued as events in the event fifo.
///  When pin code entry is in progress, this function will also call
///  handlePinCodeEntry to do pin code processing.
///
/// \return
///   Bit mapped value indicating
///       - HID_APP_ACTIVITY_NON_REPORTABLE - if any key (excluding connect button) is down. Always
///         set in pin code entry state
///       - HID_APP_ACTIVITY_REPORTABLE - if any event is queued. Always
///         set in pin code entry state
///       - HID_APP_ACTIVITY_NONE otherwise
///  As long as it is not ACTIVITY_NONE, the btlpm will be notified for low power management.
////////////////////////////////////////////////////////////////////////////////
uint8_t APP_pollActivityUser(void)
{
    uint8_t status;

    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

    mouse_poll();

     // For all other cases, return value indicating whether any event is pending or
    status = wiced_hidd_event_queue_get_num_elements(&app.eventQueue) ? HIDLINK_ACTIVITY_REPORTABLE : HIDLINK_ACTIVITY_NONE;

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_sleep_handler
////////////////////////////////////////////////////////////////////////////////
/// Summary:
///    Sleep permit query to check if sleep is allowed and sleep time
///
/// Parameters:
///  type -- quary type. It can be WICED_SLEEP_POLL_TIME_TO_SLEEP or WICED_SLEEP_POLL_SLEEP_PERMISSION
///
/// Return:
///  WICED_SLEEP_NOT_ALLOWED -- not allow to sleep
///  When WICED_SLEEP_POLL_TIME_TO_SLEEP:
///     WICED_SLEEP_MAX_TIME_TO_SLEEP or the time to sleep
///  When WICED_SLEEP_POLL_SLEEP_PERMISSION:
///     WICED_SLEEP_ALLOWED_WITH_SHUTDOWN -- allowed to sleep, but no SDS nor ePDS
///     WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN -- allowed to enter SDS/ePDS
///
////////////////////////////////////////////////////////////////////////////////
static uint32_t APP_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#if SLEEP_ALLOWED
    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            if (!(app.recoveryInProgress
                 )
               )
            {
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            }
            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
 #if SLEEP_ALLOWED > 1
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
 #else
            ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
 #endif
            break;
    }
#endif

    return ret;
}

#if 0
/////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_procErrEvtQueue
////////////////////////////////////////////////////////////////////////////////
/// This function handles event queue errors. This includes event queue overflow
/// unexpected events, missing expected events, and events in unexpected order.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem. A user defined implementation should at least
/// remove the first element in the queue if this event is an overflow event
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
static void APP_procErrEvtQueue(void)
{
    WICED_BT_TRACE("\nKSQerr");
    APP_stdErrRespWithFwHwReset();
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles error events reported by the keyscan HW. Typically
/// these would be ghost events.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem.
/// NOTE: if audio stop event was flushed out, put it back into event queue too.
////////////////////////////////////////////////////////////////////////////////
static void APP_procErrKeyscan(void)
{
    WICED_BT_TRACE("\nAPP_procErrKeyscan");

    //call base class handling
    APP_stdErrRespWithFwHwReset();

    key_clear(TRUE);
    touchpad_flush();
}
#endif

/////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_generateAndTxReports
/////////////////////////////////////////////////////////////////////////////////
/// This function provides an implementation for the generateAndTxReports() function
/// defined by the HID application. This function is only called when the active transport
/// is connected. This function performs the following actions:
///  - When pin code entry is in progress, the behavior of this function is changed.
///    It only checks and transmits the pin code report; normal event processing is
///    suspended.
///  - If the number of packets in the hardware fifo is less than the report generation
///    threshold and the event queue is not empty, this function will process events
///    by calling the event processing functions, e.g. procEvtKey() etc
///  - This function also tracks the recovery period after an error. If
///    the recovery count is non-zero, it is decremented as long as there is room
///    for one report in the transport
///
/// Parameter:
///   none
///
/// Return:
///   none
///
/////////////////////////////////////////////////////////////////////////////////
static void APP_generateAndTxReports(void)
{
    app_queue_t *curEvent;

    {
        // Normal state

        // If we are recovering from an error, decrement the recovery count as long as the transport
        // has room. Avoid the case where no event processing is done during recovery because
        // transport is full, as the failure might be a non-responding transport.
        if (app.recoveryInProgress)
        {
            // If recovery is complete, transmit any modified reports that we have been hoarding
            if (!--app.recoveryInProgress)
            {
//                key_send();
            }
        }

        // Continue report generation as long as the transport has room and we have events to process
        while ((wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) &&
               ((curEvent = (app_queue_t *)wiced_hidd_event_queue_get_current_element(&app.eventQueue)) != NULL))
        {
            // Further processing depends on the event type
            switch (curEvent->type)
            {
#if 0
                case HID_EVENT_KEY_STATE_CHANGE:
                    // process the event key. If fails, resets keys
                    if (!key_procEvtKey(curEvent->key.keyEvent.keyCode, curEvent->key.keyEvent.upDownFlag == KEY_DOWN))
                    {
                        APP_procErrKeyscan();
                    }
                    wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
                    break;
#endif

                case HID_EVENT_EVENT_FIFO_OVERFLOW:
                    WICED_BT_TRACE("\nHID_EVENT_EVENT_FIFO_OVERFLOW");
                    // Call event queue error handler
//                    APP_procErrEvtQueue();
                    wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
                    break;

                default:
                    APP_procEvtUserDefined();
                    break;
            }

            // The current event should be deleted by the event processing function.
            // Additional events may also be consumed but we don't care about that
        }

    }
}

/*******************************************************************************
 * Function Name: APP_pollReportUserActivity
 ********************************************************************************
 * Summary:
 *  This function should be called by the transport when it wants the application
 *  to poll for user activity. This function performs the following actions:
 *   - Polls for activity. If user activity is detected, events should be
 *     queued up for processing
 *   - If an unmasked user activity is detected, it passes the activity type to the
 *     transports
 *   - If the active transport is connected, requests generation of reports via
 *     generateAndTransmitReports()
 *   - Does connect button polling and informs the BT transport once the connect
 *     button has been held for the configured amount of time.
 *  Note: transport may be NULL if no transport context is required - like when
 *  none
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void APP_pollReportUserActivity(void)
{
    uint8_t activitiesDetectedInLastPoll;

    // Increment polling sequence number.
    app.pollSeqn++;

    // Check for activity. This should queue events if any user activity is detected
    activitiesDetectedInLastPoll = APP_pollActivityUser();

    // Check if the active transport is connected
    if(hidd_link_is_connected())
    {
        if((app.pollSeqn % 128) == 0)
        {
            WICED_BT_TRACE(".");
        }

        // Generate a report
        if(!bt_cfg.security_requirement_mask || hidd_link_is_encrypted())
        {
            APP_generateAndTxReports();
        }

        if (!ota_is_active())
        {
            // poll for battery monitor
            bat_poll();
        }
    }
    else
    {
        // Check if we have any user activity. If it is paired and not connected, we reconnect.
        if (activitiesDetectedInLastPoll != HIDLINK_ACTIVITY_NONE && hidd_is_paired())
        {
            // ask the transport to connect.
            hidd_link_connect();
        }
    }
}

#ifdef TESTING_USING_HCI
/////////////////////////////////////////////////////////////////////////////////
/// Demo to move mouse
/////////////////////////////////////////////////////////////////////////////////
static void APP_move_mouse_timer_timeout( uint32_t arg )
{
    #define STEP 2
    #define STEP_COUNT 50
    #define MOVE_MOUSE_DELAY 25
    static char count = STEP_COUNT;
    static char move_mouse_timer_state = 0;
    static wiced_timer_t move_mouse_timer;

    switch (move_mouse_timer_state) {
    case 0:
        wiced_init_timer( &move_mouse_timer, APP_move_mouse_timer_timeout, 0, WICED_MILLI_SECONDS_TIMER );
        move_mouse_timer_state++;
        break;
    case 1:
        mouse.reportModeReport.xMotion = STEP;
        mouse.reportModeReport.yMotion = STEP;
        break;
    case 2:
        mouse.reportModeReport.xMotion = STEP;
        mouse.reportModeReport.yMotion = -STEP;
        break;
    case 3:
        mouse.reportModeReport.xMotion = -STEP;
        mouse.reportModeReport.yMotion = -STEP;
        break;
    case 4:
        mouse.reportModeReport.xMotion = -STEP;
        mouse.reportModeReport.yMotion = STEP;
        break;
    case 5:
        move_mouse_timer_state = 1;
        count = 50;
        return;
    }

    if (!count--)
    {
        count = STEP_COUNT;
        move_mouse_timer_state++;
    }

    hidd_link_send_report(&mouse.reportModeReport, sizeof(ReportModeReport));
    wiced_start_timer(&move_mouse_timer,MOVE_MOUSE_DELAY); //timeout in ms
}

/////////////////////////////////////////////////////////////////////////////////
/// This is a callback function from keyscan when key action is detected
/////////////////////////////////////////////////////////////////////////////////
static void APP_hci_key_event(uint8_t keyCode, wiced_bool_t keyDown)
{
    if (keyCode==KEY_CONNECT)
    {
        APP_connect_button(CONNECT_KEY_INDEX, keyDown);
    }
    else if (keyCode==KEY_MOTION)
    {
        WICED_BT_TRACE("\nMove mouse command from ClientControl");
        APP_move_mouse_timer_timeout(0);
    }
}
#endif

//********************************************************************************
//********************************************************************************
void app_enter_pairing()
{
#ifdef CONNECTED_ADVERTISING_SUPPORTED
    hidd_blelink_allowDiscoverable();
#else
    hidd_link_virtual_cable_unplug();
    hidd_pairing();
#endif
}

/********************************************************************************
 * Function Name: app_setProtocol
 *******************************************************************************
 *   This function implements the setProtocol function defined by
 *   the HID application to handle "Set Protocol" messages.
 *
 * Parameters:
 *   reportType -- not used
 *   reportId -- not used
 *   payload -- pointer to new protocol
 *   payloadSize -- not used, assumed to be 1
 *
 * Return:
 *   None
 *
 *******************************************************************************/
void app_setProtocol(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize)
{
    uint8_t protocol = *((uint8_t*)payload);

//    WICED_BT_TRACE("\nNew Protocol = %d", protocol);

    APP_setProtocol(protocol);
}

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
void app_queueEvent(app_queue_t * event)
{
    wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue, &event->info, APP_QUEUE_SIZE, app.pollSeqn);
}

/*******************************************************************************
 * Function Name: app_transportStateChangeNotification
 ********************************************************************************
 * Summary:
 *    This function informs the application that the state of a link changed.
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_transportStateChangeNotification(uint32_t newState)
{
    int16_t flags;
    WICED_BT_TRACE("\nTransport state changed to ", newState);
    hidd_led_blink_stop(LED_LE_LINK);

    hidd_set_deep_sleep_allowed(WICED_FALSE);

    switch (newState) {
    case HIDLINK_LE_CONNECTED:
        WICED_BT_TRACE("connected");
        hidd_led_on(LED_LE_LINK);

        hidd_blelink_enable_poll_callback(WICED_TRUE);

        if(app.firstTransportStateChangeNotification)
        {
            //Wake up from HID Off and already have a connection then allow HID Off in 1 second
            //This will allow time to send a key press.
            //To do need to check if key event is in the queue at lpm query
            hidd_deep_sleep_not_allowed(1000); // 1 second. timeout in ms
        }
        else
        {
            //We connected after power on reset or HID off recovery.
            //Start 20 second timer to allow time to setup connection encryption
            //before allowing HID Off/Micro-BCS.
            hidd_deep_sleep_not_allowed(20000); //20 seconds. timeout in ms

        }
        break;

    case HIDLINK_LE_DISCONNECTED:
        WICED_BT_TRACE("disconnected");
        hidd_led_off(LED_LE_LINK);
        hidd_blelink_enable_poll_callback(WICED_FALSE);
        if (app.enter_pairing_pending)
        {
            app.enter_pairing_pending = 0;
            app_enter_pairing();
        }
        break;

    case HIDLINK_LE_DISCOVERABLE:
        WICED_BT_TRACE("discoverable");
        hidd_led_blink(LED_LE_LINK, 0, 500);
        break;

    case HIDLINK_LE_RECONNECTING:
        WICED_BT_TRACE("reconnecting");
        hidd_led_blink(LED_LE_LINK, 0, 200);     // faster blink LINK line to indicate reconnecting
        break;

    case HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED:
    case HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED:
        hidd_set_deep_sleep_allowed(WICED_TRUE);
        break;
    }

    app.firstTransportStateChangeNotification = 0;
}

/*******************************************************************************
 * sleep configuration
 *******************************************************************************/
wiced_sleep_config_t    hidd_link_sleep_config = {
    . sleep_mode            = WICED_SLEEP_MODE_NO_TRANSPORT,  //sleep_mode
    .host_wake_mode         = 0,                              //host_wake_mode
    .device_wake_mode       = 0,                              //device_wake_mode
    .device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO | WICED_SLEEP_WAKE_SOURCE_KEYSCAN | WICED_SLEEP_WAKE_SOURCE_QUAD,  //device_wake_source
    .device_wake_gpio_num   = 255,                            //must set device_wake_gpio_num to 255 for WICED_SLEEP_MODE_NO_TRANSPORT
    .sleep_permit_handler   = APP_sleep_handler,              //sleep_permit_handler
#if defined(CYW20819A1) || defined(CYW20820A1)
    .post_sleep_cback_handler=NULL,                           //post_sleep_handler
#endif
};

/*******************************************************************************
 * Callback functions
 *******************************************************************************/
static hidd_link_callback_t appCallbacks =
{
    .p_app_poll_user_activities                 = APP_pollReportUserActivity,
    .p_app_connection_failed_notification       = NULL,

#ifdef SUPPORT_CODE_ENTRY
    .p_app_enter_pincode_entry_mode             = NULL,
    .p_app_enter_passcode_entry_mode            = NULL,
    .p_app_exit_pin_and_passcode_entry_mode     = NULL,
#endif
    .p_app_get_idle                             = NULL,
    .p_app_set_idle                             = NULL,
    .p_app_get_protocol                         = APP_getProtocol,
    .p_app_set_protocol                         = APP_setProtocol,
    .p_app_get_report                           = NULL,
    .p_app_set_report                           = NULL,
    .p_app_rx_data                              = NULL,
};

/*******************************************************************************
 * Function Name: app_start()
 ********************************************************************************
 * Summary: This is application start function. After system is up, when the
 *          bt management calls with BTM_ENABLED_EVT, this function is called to
 *          start application
 *
 * Parameters:
 *  none
 *
 * Return:
 *  WICED_BT_SUCCESS -- if initialization is okay and ready to start;
 *                      otherwise, should return the error code defined in wiced_result_t
 *
 *******************************************************************************/
wiced_result_t app_start(void)
{
    WICED_BT_TRACE("\napp_start");

    // allocate necessary memory and initialize event queue
    wiced_hidd_event_queue_init(&app.eventQueue, (uint8_t *)&app.events, APP_QUEUE_SIZE, APP_QUEUE_MAX);

    // register applicaton callbacks
    hidd_register_app_callback(&appCallbacks);

    /* Client Control key detected callback */
    hci_control_register_key_handler(APP_hci_key_event);

    /* transport init */
    bt_init();

    hidd_sleep_configure(&hidd_link_sleep_config);

    /* component/peripheral init */
    bat_init(APP_shutdown);
    mouse_init();
    hidd_link_init();

    wiced_hal_mia_enable_mia_interrupt(TRUE);
    wiced_hal_mia_enable_lhl_interrupt(TRUE);//GPIO interrupt

    WICED_BT_TRACE("\nFree RAM bytes=%d bytes", wiced_memory_get_free_bytes());

    return WICED_BT_SUCCESS;
}

/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_srv_switchable SwitchaBLE Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief SwitchaBLE Service module.
 *
 * @details This module implements the SwitchaBLE Service with the Battery Level characteristic.
 *          During initialization it adds the SwitchaBLE Service and Battery Level characteristic
 *          to the BLE stack dataswitchablee. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the SwitchaBLE Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_switchable_battery_level_update() function.
 *          If an event handler is supplied by the application, the SwitchaBLE Service will
 *          generate SwitchaBLE Service events to the application.
 *
 * @note The application must propagate BLE stack events to the SwitchaBLE Service module by calling
 *       ble_switchable_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef BLE_switchable_H__
#define BLE_switchable_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define SWITCHABLE_UUID_BASE {0x26, 0xC1, 0xE1, 0x5C, 0xFB, 0x09, 0x37, 0x0D, 0x9D, 0x92, 0x47, 0x1B, 0x00, 0x00, 0x00, 0x00}
#define SWITCHABLE_UUID_SERVICE       0x6D59
#define SWITCHABLE_UUID_LIGHT_CHAR    0x6D5A    // Read, Write
#define SWITCHABLE_UUID_ALRM1_CHAR    0x6D5C    // Read, Notify
//#define SWITCHABLE_UUID_TIMER1_CHAR   0x6D5C    // Read, Write
#define SWITCHABLE_UUID_ALRM2_CHAR    0x6D5D    // Read, Notify
#define SWITCHABLE_UUID_TIMER2_CHAR   0x6D5E    // Read, Write
#define SWITCHABLE_UUID_BUTTON_CHAR   0x6D5B

// Forward declaration of the ble_switchable_t type. 
typedef struct ble_switchable_s ble_switchable_t;

/**@brief SwitchaBLE Service event handler type. */
typedef void (*ble_switchable_evt_handler_t) (ble_switchable_t * p_switchable, uint8_t new_state);

/**@brief SwitchaBLE Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_switchable_evt_handler_t    light_write_handler;
    ble_switchable_evt_handler_t    alarm1_write_handler;
    ble_switchable_evt_handler_t    alarm1_timeout_handler;
    ble_switchable_evt_handler_t    alarm2_write_handler;
    ble_switchable_evt_handler_t    alarm2_timeout_handler;                   
} ble_switchable_init_t;


/**@brief SwitchaBLE Service structure. This contains various status information for the service. */
typedef struct ble_switchable_s
{         
    uint16_t                     service_handle;                 
    ble_gatts_char_handles_t     light_char_handles;          
    ble_gatts_char_handles_t     button_char_handles;
    ble_gatts_char_handles_t	 alarm1_char_handles;
    ble_gatts_char_handles_t	 alarm2_char_handles;          
    uint8_t                      uuid_type;
    uint8_t                      current_light_state;             
    uint16_t                     conn_handle;  
    bool                         is_notifying;
    ble_switchable_evt_handler_t light_write_handler;
    ble_switchable_evt_handler_t alarm1_write_handler;
    ble_switchable_evt_handler_t alarm1_timeout_handler;
    ble_switchable_evt_handler_t alarm2_write_handler;
    ble_switchable_evt_handler_t alarm2_timeout_handler;
} ble_switchable_t;


/**@brief Initialize the SwitchaBLE Service.
 *
 * @param[out]  p_switchable       
 * @param[in]   p_switchable_init  
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_switchable_init(ble_switchable_t * p_switchable, const ble_switchable_init_t * p_switchable_init);

/**@brief SwitchaBLE Service BLE stack event handler.
 *
 * @details Handles all events from the BLE stack of interest to the SwitchaBLE Service.
 *
 * @param[in]   p_switchable      SwitchaBLE Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_switchable_on_ble_evt(ble_switchable_t * p_switchable, ble_evt_t * p_ble_evt);

/**@brief Handler to notify the SwitchaBLE service on button presses.
 *
 * @param[in]   p_switchable      SwitchaBLE Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
uint32_t ble_switchable_on_button_change(ble_switchable_t * p_switchable, uint8_t button_state);

/**@brief Handler to notify the SwitchaBLE service on light change.
 *
 * @param[in]   p_switchable      SwitchaBLE Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
uint32_t ble_switchable_on_light_change(ble_switchable_t * p_switchable, uint8_t light_state);

#endif // BLE_switchable_H__

/** @} */

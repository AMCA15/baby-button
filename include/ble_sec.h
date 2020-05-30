/**
 * @file ble_sec.h
 * @author Anderson Contreras
 * @brief BLE security and authentication
 * @version 0.1
 * @date 2020-05-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */


#ifndef BLE_SEC_H__
#define BLE_SEC_H__

#include <ble.h>
#include <ble_gap.h>

#define PASSKEY                "123456"                  /**< 6-digit ASCII string (digit 0..9 only, no NULL termination) passkey to be used during pairing.*/
#define PASSKEY_LENGTH          6                        /**< Length of pass-key received by the stack for display. */

/**@brief Set the SoftDevice to use a static passkey defined on PASSKEY
 */
void set_static_passkey(void);

/**@brief Set the SoftDevice to use a random passkey
 */
void set_random_passkey(void);

/**@brief Print the passkey
 * 
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void print_passkey(ble_evt_t const * p_ble_evt);

#endif // BLE_SEC_H__
/**
 * @file ble_sec.c
 * @author Anderson Contreras
 * @brief BLE security and authentication
 * @version 0.1
 * @date 2020-05-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <ble_sec.h>
#include <app_error.h>
#include <string.h>
#include <nrf_log.h>
#include <string.h>


/**@brief Set the SoftDevice to use a static passkey defined on PASSKEY
 */
void set_static_passkey(void) {
    ret_code_t err_code;
    ble_opt_t ble_opt;

    memset(&ble_opt, 0, sizeof(ble_opt));

    ble_opt.gap_opt.passkey.p_passkey = (uint8_t *) PASSKEY;
    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
    APP_ERROR_CHECK(err_code);
}

/**@brief Set the SoftDevice to use a random passkey
 */
void set_random_passkey(void) {
    ret_code_t err_code;
    ble_opt_t ble_opt;

    memset(&ble_opt, 0, sizeof(ble_opt));

    err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
    APP_ERROR_CHECK(err_code);
}

/**@brief Print the passkey
 * 
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void print_passkey(ble_evt_t const * p_ble_evt) {
    char passkey[PASSKEY_LENGTH + 1];
    memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
    passkey[PASSKEY_LENGTH] = 0;

    NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
}
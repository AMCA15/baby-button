/**
 * @file buttonless.h
 * @author Anderson Contreras <acontreras@aimonkey.info>
 * @brief DFU Buttonless related functions
 * @version 0.1
 * @date 2020-05-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef BUTTONLESS_H__
#define BUTTONLESS_H__

#include <ble_dfu.h>
#include <ble_advertising.h>

#include <nrf_dfu_ble_svci_bond_sharing.h>
#include <nrf_svci_async_function.h>
#include <nrf_svci_async_handler.h>

/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event);

/**
 * @brief This is a workaround for get the m_adversiting from the main.c file
 * 
 * @param p_adv Pointer to the adversiting structure
 */
void ble_dfu_get_adv(ble_advertising_t * p_adv);

#endif // BUTTONLESS_H__
/**
 * @file ble_mas.c
 * @author Anderson Contreras <acontreras@aimonkey.info>
 * @brief 
 * @version 0.1
 * @date 2020-04-22
 * 
 * @copyright Copyright (c) 2020
 * 
 */


#include "ble_mas.h"
#include "nrf_log.h"
#include "ble_conn_state.h"

#define BLE_UUID_MAS_ACC_CHARACTERISTIC 0x0002               /**< The UUID of the Accelerometer Characteristic. */
#define BLE_UUID_MAS_GYR_CHARACTERISTIC 0x0003               /**< The UUID of the Gyroscope Characteristic. */

#define BLE_MAS_MAX_ACC_CHAR_LEN        6                    /**< Maximum length of the Acceletometer Characteristic (in bytes). */
#define BLE_MAS_MAX_GYR_CHAR_LEN        6                    /**< Maximum length of the Acceletometer Characteristic (in bytes). */

#define MAS_BASE_UUID                  {{0x20, 0xBB, 0xBA, 0x22, 0x68, 0x55, 0x41, 0x98, 0xBA, 0xAA, 0xA5, 0x68, 0x57, 0xA6, 0x25, 0xE4}} /**< Used vendor specific UUID. */


#define ACC_CHAR_USER_DESCR_VALUE    "Accelerometer"           /**< The User Descriptor value of the Accelerometer Characteristic. */
#define GYR_CHAR_USER_DESCR_VALUE    "Gyroscope"               /**< The User Descriptor value of the Gyroscope Characteristic. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_mas       Monitor Activity Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_mas_t * p_mas, ble_evt_t const * p_ble_evt)
{
    p_mas->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_mas       Monitor Activity Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_mas_t * p_mas, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_mas->conn_handle = BLE_CONN_HANDLE_INVALID;
}



/**@brief Function for adding the Accelerometer characteristic.
 *
 * @param[in]   p_bas        Monitor Activity Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t accelerometer_char_add(ble_mas_t * p_mas, const ble_mas_init_t * p_mas_init) {
    ret_code_t err_code;
    ble_add_char_params_t add_char_params;
    ble_add_char_user_desc_t add_char_user_desc;

    // Add the Accelerometer Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_MAS_ACC_CHARACTERISTIC;
    add_char_params.uuid_type                = p_mas->uuid_type;
    add_char_params.max_len                  = BLE_MAS_MAX_ACC_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = p_mas_init->support_acc_notification;
    add_char_params.read_access              = p_mas_init->acc_rd_sec;
    add_char_params.cccd_write_access        = p_mas_init->acc_cccd_wr_sec;
    add_char_params.p_user_descr             = &add_char_user_desc;

    // Add the Accelerometer Characteristic User Descriptor.
    memset(&add_char_user_desc, 0, sizeof(add_char_user_desc));
    add_char_user_desc.p_char_user_desc      = (uint8_t *) ACC_CHAR_USER_DESCR_VALUE;
    add_char_user_desc.size                  = strlen((char *) ACC_CHAR_USER_DESCR_VALUE);
    add_char_user_desc.max_size              = add_char_user_desc.size;
    add_char_user_desc.char_props.read       = 1;
    add_char_user_desc.read_access           = SEC_OPEN;

    err_code = characteristic_add(p_mas->service_handle, &add_char_params, &(p_mas->accelerometer_handles));
    return err_code;
}


/**@brief Function for adding the Gyroscope characteristic.
 *
 * @param[in]   p_bas        Monitor Activity Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t gyroscope_char_add(ble_mas_t * p_mas, const ble_mas_init_t * p_mas_init) {
    ret_code_t err_code;
    ble_add_char_params_t add_char_params;
    ble_add_char_user_desc_t add_char_user_desc;

    // Add the Gyroscope Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_MAS_GYR_CHARACTERISTIC;
    add_char_params.uuid_type                = p_mas->uuid_type;
    add_char_params.max_len                  = BLE_MAS_MAX_GYR_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = p_mas_init->support_gyr_notification;
    add_char_params.read_access              = p_mas_init->gyr_rd_sec;
    add_char_params.cccd_write_access        = p_mas_init->gyr_cccd_wr_sec;
    add_char_params.p_user_descr             = &add_char_user_desc;

    // Add the Gyroscope Characteristic User Descriptor.
    memset(&add_char_user_desc, 0, sizeof(add_char_user_desc));
    add_char_user_desc.p_char_user_desc      = (uint8_t *) GYR_CHAR_USER_DESCR_VALUE;
    add_char_user_desc.size                  = strlen((char *) GYR_CHAR_USER_DESCR_VALUE);
    add_char_user_desc.max_size              = add_char_user_desc.size;
    add_char_user_desc.char_props.read       = 1;
    add_char_user_desc.read_access           = SEC_OPEN;

    err_code = characteristic_add(p_mas->service_handle, &add_char_params, &(p_mas->gyroscope_handles));
    return err_code;
}


void ble_mas_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_mas_t * p_mas = (ble_mas_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_mas, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_mas, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_INFO("BLE_GATTS_EVT_WRITE");
            // TODO: Implement on_write function
            // on_write(p_mas, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


ret_code_t ble_mas_init(ble_mas_t * p_mas, const ble_mas_init_t * p_mas_init)
{
    VERIFY_PARAM_NOT_NULL(p_mas);
    VERIFY_PARAM_NOT_NULL(p_mas_init);

    ret_code_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t mas_base_uuid = MAS_BASE_UUID;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&mas_base_uuid, &p_mas->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_mas->uuid_type;
    ble_uuid.uuid = BLE_UUID_MONITOR_ACTIVITY_SERVICE;


    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_mas->service_handle);
    VERIFY_SUCCESS(err_code);

    err_code = accelerometer_char_add(p_mas, p_mas_init);
    VERIFY_SUCCESS(err_code);

    if(p_mas_init->support_gyroscope){
        err_code = gyroscope_char_add(p_mas, p_mas_init);
    }

    return err_code;
}


uint32_t ble_mas_accelerometer_measurement_send(ble_mas_t * p_mas, int16_t acc_x, uint16_t acc_y, uint16_t acc_z)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_mas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_mas[BLE_MAS_MAX_ACC_CHAR_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len = 0;
        len += uint16_big_encode(acc_x, &encoded_mas[len]);
        len += uint16_big_encode(acc_y, &encoded_mas[len]);
        len += uint16_big_encode(acc_z, &encoded_mas[len]);

        len = sizeof(encoded_mas);

        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_mas->accelerometer_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_mas;

        err_code = sd_ble_gatts_hvx(p_mas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


uint32_t ble_mas_gyroscope_measurement_send(ble_mas_t * p_mas, int16_t gyr_x, uint16_t gyr_y, uint16_t gyr_z)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_mas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_mas[BLE_MAS_MAX_GYR_CHAR_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len = 0;
        len += uint16_big_encode(gyr_x, &encoded_mas[len]);
        len += uint16_big_encode(gyr_y, &encoded_mas[len]);
        len += uint16_big_encode(gyr_z, &encoded_mas[len]);

        len = sizeof(encoded_mas);

        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_mas->gyroscope_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_mas;

        err_code = sd_ble_gatts_hvx(p_mas->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}
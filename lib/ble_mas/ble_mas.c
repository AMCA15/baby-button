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

#define BLE_UUID_MAS_ACC_CHARACTERISTIC                  0x0002     /**< The UUID of the Accelerometer Characteristic. */
#define BLE_UUID_MAS_GYR_CHARACTERISTIC                  0x0003     /**< The UUID of the Gyroscope Characteristic. */
#define BLE_UUID_MAS_FEAT_CHARACTERISTIC                 0x0004     /**< The UUID of the Feature Characteristic. */
#define BLE_UUID_MONITOR_ACTIVITY_CONTROL_POINT_CHAR     0x0100     /**< Monitor Activity Control Point characteristic UUID. */

#define BLE_MAS_MAX_ACC_CHAR_LEN        96                    /**< Maximum length of the Acceletometer Characteristic (in bytes). */
#define BLE_MAS_MAX_GYR_CHAR_LEN        96                    /**< Maximum length of the Gyroscope Characteristic (in bytes). */
#define BLE_MAS_MAX_FEAT_CHAR_LEN       1                    /**< Maximum length of the Feature Characteristic (in bytes). */
#define BLE_MACP_MAX_GYR_CHAR_LEN       1                    /**< Maximum length of the Monitor Activity Control Point Characteristic (in bytes). */

#define MAS_BASE_UUID                  {{0x20, 0xBB, 0xBA, 0x22, 0x68, 0x55, 0x41, 0x98, 0xBA, 0xAA, 0xA5, 0x68, 0x57, 0xA6, 0x25, 0xE4}} /**< Used vendor specific UUID. */


#define ACC_CHAR_USER_DESCR_VALUE    "Accelerometer"           /**< The User Descriptor value of the Accelerometer Characteristic. */
#define GYR_CHAR_USER_DESCR_VALUE    "Gyroscope"               /**< The User Descriptor value of the Gyroscope Characteristic. */
#define FEAT_CHAR_USER_DESCR_VALUE   "Features"                /**< The User Descriptor value of the Features Characteristic. */
#define MACP_CHAR_USER_DESCR_VALUE   "Control Point"           /**< The User Descriptor value of the Monitor Activity Control Point Characteristic. */


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


/**@brief Function for handling the write events to the Blood Pressure Measurement characteristic.
 *
 * @param[in]   p_mas         Monitor Activity Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_macp_write(ble_mas_t * p_mas, ble_gatts_evt_write_t const * p_evt_write)
{
    p_mas->macp_evt_handler(p_evt_write->data, p_evt_write->len);
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_mas       Monitor Activity Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_mas_t * p_mas, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_mas->macp_handles.value_handle)
    {
        on_macp_write(p_mas, p_evt_write);
    }
}


/**@brief Function for adding the Accelerometer characteristic.
 *
 * @param[in]   p_mas        Monitor Activity Service structure.
 * @param[in]   p_mas_init   Information needed to initialize the service.
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
 * @param[in]   p_mas        Monitor Activity Service structure.
 * @param[in]   p_mas_init   Information needed to initialize the service.
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


/**@brief Function for adding the Features characteristic.
 *
 * @param[in]   p_mas        Monitor Activity Service structure.
 * @param[in]   p_mas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t features_char_add(ble_mas_t * p_mas, const ble_mas_init_t * p_mas_init) {
    ret_code_t err_code;
    ble_add_char_params_t add_char_params;
    ble_add_char_user_desc_t add_char_user_desc;

    // Add the Gyroscope Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_MAS_FEAT_CHARACTERISTIC;
    add_char_params.uuid_type                = p_mas->uuid_type;
    add_char_params.max_len                  = BLE_MAS_MAX_FEAT_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = p_mas_init->support_feat_notification;
    add_char_params.read_access              = p_mas_init->feat_rd_sec;
    add_char_params.cccd_write_access        = p_mas_init->feat_cccd_wr_sec;
    add_char_params.p_user_descr             = &add_char_user_desc;

    // Add the Gyroscope Characteristic User Descriptor.
    memset(&add_char_user_desc, 0, sizeof(add_char_user_desc));
    add_char_user_desc.p_char_user_desc      = (uint8_t *) FEAT_CHAR_USER_DESCR_VALUE;
    add_char_user_desc.size                  = strlen((char *) FEAT_CHAR_USER_DESCR_VALUE);
    add_char_user_desc.max_size              = add_char_user_desc.size;
    add_char_user_desc.char_props.read       = 1;
    add_char_user_desc.read_access           = SEC_OPEN;

    err_code = characteristic_add(p_mas->service_handle, &add_char_params, &(p_mas->features_handles));
    return err_code;
}


/**@brief Function for adding the Monitor Activity Control Point characteristic.
 *
 * @param[in]   p_mas        Monitor Activity Service structure.
 * @param[in]   p_mas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static ret_code_t macp_char_add(ble_mas_t * p_mas, const ble_mas_init_t * p_mas_init) {
    ret_code_t err_code;
    ble_add_char_params_t add_char_params;
    ble_add_char_user_desc_t add_char_user_desc;

    // Add the Monitor Activity Control Point Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = BLE_UUID_MONITOR_ACTIVITY_CONTROL_POINT_CHAR;
    add_char_params.uuid_type                = p_mas->uuid_type;
    add_char_params.max_len                  = BLE_MACP_MAX_GYR_CHAR_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.write         = 1;
    add_char_params.write_access             = p_mas_init->macp_wr_sec;
    add_char_params.p_user_descr             = &add_char_user_desc;

    // Add the Monitor Activity Control Point Characteristic User Descriptor.
    memset(&add_char_user_desc, 0, sizeof(add_char_user_desc));
    add_char_user_desc.p_char_user_desc      = (uint8_t *) MACP_CHAR_USER_DESCR_VALUE;
    add_char_user_desc.size                  = strlen((char *) MACP_CHAR_USER_DESCR_VALUE);
    add_char_user_desc.max_size              = add_char_user_desc.size;
    add_char_user_desc.char_props.read       = 1;
    add_char_user_desc.read_access           = SEC_OPEN;

    err_code = characteristic_add(p_mas->service_handle, &add_char_params, &(p_mas->macp_handles));
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
            on_write(p_mas, p_ble_evt);
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

    // Initialize service structure
    p_mas->evt_handler      = p_mas_init->evt_handler;
    p_mas->macp_evt_handler = p_mas_init->macp_evt_handler;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_mas->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the Accelerometer characteristic.
    err_code = accelerometer_char_add(p_mas, p_mas_init);
    VERIFY_SUCCESS(err_code);

    // Add the Gyroscope characteristic.
    if(p_mas_init->support_gyroscope){
        err_code = gyroscope_char_add(p_mas, p_mas_init);
        VERIFY_SUCCESS(err_code);
    }

    // Add the Features characteristic.
    if(p_mas_init->support_features){
        err_code = features_char_add(p_mas, p_mas_init);
        VERIFY_SUCCESS(err_code);
    }

    // Add the Control Point characteristic.
    if(p_mas_init->macp_evt_handler != NULL) {
        err_code = macp_char_add(p_mas, p_mas_init);
    }

    return err_code;
}


uint32_t ble_mas_accelerometer_measurement_send(ble_mas_t * p_mas, uint16_t acc_x, uint16_t acc_y, uint16_t acc_z)
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


uint32_t ble_mas_gyroscope_measurement_send(ble_mas_t * p_mas, uint16_t gyr_x, uint16_t gyr_y, uint16_t gyr_z)
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


uint32_t ble_mas_accelerometer_bulk_send(ble_mas_t * p_mas, uint16_t * acc_x, uint16_t * acc_y, uint16_t * acc_z, uint8_t length)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_mas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_mas[BLE_MAS_MAX_ACC_CHAR_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        // len = 0;
        // len += uint16_big_encode(acc_x, &encoded_mas[len]);
        // len += uint16_big_encode(acc_y, &encoded_mas[len]);
        // len += uint16_big_encode(acc_z, &encoded_mas[len]);


        // for(uint8_t i=0; i < length; i++) {
        //     NRF_LOG_RAW_INFO("%04x, %04x, %04x, ", acc_x[i], acc_y[i], acc_z[i]);
        // }
        // NRF_LOG_RAW_INFO("\n");

        len = 0;
        for(uint8_t i=0; i < length; i++) {
            len += uint16_big_encode(acc_x[i], &encoded_mas[len]);
            len += uint16_big_encode(acc_y[i], &encoded_mas[len]);
            len += uint16_big_encode(acc_z[i], &encoded_mas[len]);
        }
        
        // for(uint8_t i=0, len=0; i<length;i++, len+=6){
        //     NRF_LOG_RAW_INFO("[e] %02x %02x, %02x %02x, %02x %02x\n", encoded_mas[len], encoded_mas[len+1], encoded_mas[len+2],
        //                                                     encoded_mas[len+3], encoded_mas[len+4], encoded_mas[len+5]);
        // }

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


uint32_t ble_mas_gyroscope_bulk_send(ble_mas_t * p_mas, uint16_t * gyr_x, uint16_t * gyr_y, uint16_t * gyr_z, uint8_t length)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_mas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_mas[BLE_MAS_MAX_GYR_CHAR_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        // len = 0;
        // len += uint16_big_encode(gyr_x, &encoded_mas[len]);
        // len += uint16_big_encode(gyr_y, &encoded_mas[len]);
        // len += uint16_big_encode(gyr_z, &encoded_mas[len]);


        len = 0;
        for(uint8_t i=0; i < length; i++) {
            len += uint16_big_encode(gyr_x[i], &encoded_mas[len]);
            len += uint16_big_encode(gyr_y[i], &encoded_mas[len]);
            len += uint16_big_encode(gyr_z[i], &encoded_mas[len]);
        }


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


uint32_t ble_mas_features_send(ble_mas_t * p_mas, uint8_t data)
{
    uint32_t err_code;

    // Send value if connected and notifying
    if (p_mas->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_mas[BLE_MAS_MAX_FEAT_CHAR_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        encoded_mas[0] = data;

        len = sizeof(encoded_mas);

        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_mas->features_handles.value_handle;
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
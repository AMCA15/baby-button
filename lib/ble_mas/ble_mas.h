/**
 * @file ble_mas.h
 * @author Anderson Contreras <acontreras@aimonkey.info>
 * @brief 
 * @version 0.1
 * @date 2020-04-22
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef BLE_MAS_H__
#define BLE_MAS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_UUID_MONITOR_ACTIVITY_SERVICE       0x0001


/**@brief Macro for defining a ble_mas instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_MAS_DEF(_name)                          \
    static ble_mas_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                         BLE_MAS_BLE_OBSERVER_PRIO, \
                         ble_mas_on_ble_evt,        \
                         &_name)


/**@brief Monitor Activity Service event type. */
typedef enum
{
    BLE_MAS_EVT_NOTIFICATION_ENABLED, /**< Monitor Activity value notification enabled event. */
    BLE_MAS_EVT_NOTIFICATION_DISABLED /**< Monitor Activity value notification disabled event. */
} ble_mas_evt_type_t;


/**@brief Monitor Activity Service event. */
typedef struct
{
    ble_mas_evt_type_t evt_type;    /**< Type of event. */
    uint16_t           conn_handle; /**< Connection handle. */
} ble_mas_evt_t;


// Forward declaration of the ble_mas_t type.
typedef struct ble_mas_s ble_mas_t;


/**@brief Monitor Activity Service event handler type. */
typedef void (*ble_mas_evt_handler_t) (ble_mas_t * p_mas, ble_mas_evt_t * p_evt);


/**@brief Monitor Activity Control Point event handler type. */
typedef void (*ble_macp_evt_handler_t) (uint8_t * data, uint8_t size);


/**@brief Monitor Activity Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_mas_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    ble_macp_evt_handler_t macp_evt_handler;               /**< Event handler to be called for handling events in the Monitor Activity Control Point Characteristic. */
    bool                   support_acc_notification;       /**< TRUE if notification of Accelerometer measurement is supported. */
    bool                   support_gyr_notification;       /**< TRUE if notification of Gyroscope measurement is supported. */
    bool                   support_feat_notification;      /**< TRUE if notification of Features is supported. */
    bool                   support_gyroscope;              /**< TRUE if Gyroscope measurement is supported. */
    bool                   support_features;               /**< TRUE if Features is supported. */
    security_req_t         acc_cccd_wr_sec;                /**< Security requirement for writing the Acceleromter characteristic CCCD. */
    security_req_t         acc_rd_sec;                     /**< Security requirement for reading the Accelerometer characteristic value. */
    security_req_t         gyr_cccd_wr_sec;                /**< Security requirement for writing the Gyroscope characteristic CCCD. */
    security_req_t         gyr_rd_sec;                     /**< Security requirement for reading the Gyroscope characteristic value. */
    security_req_t         feat_cccd_wr_sec;               /**< Security requirement for writing the Features characteristic CCCD. */
    security_req_t         feat_rd_sec;                    /**< Security requirement for reading the Features characteristic value. */
    security_req_t         macp_wr_sec;                    /**< Security requirement for writing the Monitor Activity Control Point characteristic CCCD. */
} ble_mas_init_t;


/**@brief Monitor Activity Service structure. This contains various status information for the service. */
struct ble_mas_s
{
    uint8_t                  uuid_type;                        /**< UUID type for Monitor Activity Service Base UUID. */
    ble_mas_evt_handler_t    evt_handler;                      /**< Event handler to be called for handling events in the Monitor Activity Service. */
    ble_macp_evt_handler_t   macp_evt_handler;                 /**< Event handler to be called for handling events in the Monitor Activity Control Point Characteristic. */
    ble_gatts_char_handles_t accelerometer_handles;            /**< Handles related to the Accelerometer characteristic. */
    ble_gatts_char_handles_t gyroscope_handles;                /**< Handles related to the Gyroscope characteristic. */
    ble_gatts_char_handles_t features_handles;                 /**< Handles related to the Features characteristic. */
    ble_gatts_char_handles_t macp_handles;                     /**< Handles related to the Monitor Activity Control Point characteristic. */
    uint16_t                 service_handle;                   /**< Handle of Monitor Activity Service (as provided by the BLE stack). */
    uint16_t                 conn_handle;                      /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                     is_acc_notification_supported;    /**< TRUE if notification of Accelerometer is supported. */
    bool                     is_gyr_notification_supported;    /**< TRUE if notification of Gyroscope is supported. */
};


/**@brief Function for initializing the Monitor Activity Service.
 *
 * @param[out]  p_mas       Monitor Activity Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_mas_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
ret_code_t ble_mas_init(ble_mas_t * p_mas, const ble_mas_init_t * p_mas_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Monitor Activity Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Monitor Activity Service structure.
 */
void ble_mas_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending accelerometer measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed an acceleration measurement.
 *          If notification has been enabled, the acceleration measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_mas                    Monitor Activity Service structure.
 * @param[in]   acc_x                    New acceleration measurement on X axis.
 * @param[in]   acc_y                    New acceleration measurement on Y axis.
 * @param[in]   acc_z                    New acceleration measurement on Z axis.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mas_accelerometer_measurement_send(ble_mas_t * p_mas, int16_t acc_x, uint16_t acc_y, uint16_t acc_z);


/**@brief Function for sending gyroscope measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed an angular velocity measurement.
 *          If notification has been enabled, the angular velocity measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_mas                    Monitor Activity Service structure.
 * @param[in]   gyr_x                    New angular velocity measurement on X axis.
 * @param[in]   gyr_y                    New angular velocity measurement on Y axis.
 * @param[in]   gyr_z                    New angular velocity measurement on Z axis.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mas_gyroscope_measurement_send(ble_mas_t * p_mas, int16_t gyr_x, uint16_t gyr_y, uint16_t gyr_z);


/**@brief Function for sending accelerometer measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed an acceleration measurement.
 *          If notification has been enabled, the acceleration measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_mas                    Monitor Activity Service structure.
 * @param[in]   acc_x                    New acceleration measurement on X axis.
 * @param[in]   acc_y                    New acceleration measurement on Y axis.
 * @param[in]   acc_z                    New acceleration measurement on Z axis.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mas_accelerometer_bulk_send(ble_mas_t * p_mas, int16_t *acc_x, uint16_t *acc_y, uint16_t *acc_z, uint8_t length);


/**@brief Function for sending gyroscope measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed an angular velocity measurement.
 *          If notification has been enabled, the angular velocity measurement data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_mas                    Monitor Activity Service structure.
 * @param[in]   gyr_x                    New angular velocity measurement on X axis.
 * @param[in]   gyr_y                    New angular velocity measurement on Y axis.
 * @param[in]   gyr_z                    New angular velocity measurement on Z axis.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mas_gyroscope_bulk_send(ble_mas_t * p_mas, int16_t *gyr_x, uint16_t *gyr_y, uint16_t *gyr_z, uint8_t length);


/**@brief Function for sending features data if notification has been enabled.
 *
 * @details The application calls this function after having performed any features data update.
 *          If notification has been enabled, the features data is encoded and sent to
 *          the client.
 *
 * @param[in]   p_mas                    Monitor Activity Service structure.
 * @param[in]   data                     New features data.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mas_features_send(ble_mas_t * p_mas, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif // BLE_MAS_H__
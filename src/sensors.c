/**
 * @file sensors.c
 * @author Anderson Contreras
 * @brief Function for handling sensors, peripherals and battery stuffs.
 * @version 0.1
 * @date 2020-04-29
 *
 * @copyright Copyright (c) 2020
 *
 */


#include "sensors.h"
#include <nrf_log.h>
#include <nrfx_saadc.h>
#include <nrfx_twim.h>
#include <nrf_gpio.h>
#include <ble.h>
#include <ble_bas.h>
#include "ble_mas.h"
#include "SparkFunLSM9DS1.h"

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                  /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS  270                /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                   1024               /**< Maximum digital value for 10-bit ADC conversion. */

#define SDA_PIN                         6                  /**< SDA pin used by the TWI. */
#define SCL_PIN                         8                  /**< SCL pin used by the TWI. */

#define LSM9DS1_SA0                     1
#define LSM9DS1_SA1                     0
// TODO: We need to tuning the Inactivity Threshold
#define LSM9DS1_INACTIVITY_THS          40                 /**< Set the LSM9DS1 Inactivity threshold. Actual value unknow, the documentation doesn't say it. */
#define LSM9DS1_INACTIVITY_DUR          255                /**< Set the LSM9DS1 Inactivity duration. Actual value unknow, the documentation doesn't say it. */

#define INT1_AG      NRF_GPIO_PIN_MAP(1,11)                /**< INT1_AG pin of the LSM9DS1. */
#define INT2_AG      NRF_GPIO_PIN_MAP(1,10)                /**< INT2_AG pin of the LSM9DS1. */
#define INT_M        NRF_GPIO_PIN_MAP(1,13)                /**< INT_M   pin of the LSM9DS1. */
#define DRDY_M       NRF_GPIO_PIN_MAP(1,15)                /**< DRDY_M  pin of the LSM9DS1. */


/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

// Operations for Monitor Activity Control Point
enum {
    OP_MACP_CALIBRATE_ALL,
    OP_MACP_CALIBRATE_AG,
    OP_MACP_CALIBRATE_M,
} op_macp_t;

static nrf_saadc_value_t adc_buf;
static ble_bas_t * p_bas;

static nrfx_twim_t nrfx_twim = NRFX_TWIM_INSTANCE(0);
static lsm9ds1_t lsm9ds1;
static ble_mas_t * p_mas;

// Feature data Struct
static union features_data_t {
    struct bit_t {
        uint8_t calibrated_ag : 1;
        uint8_t calibrated_m  : 1;
        uint8_t inactivity    : 1;
    } bit;
    uint8_t byte;
} features_data;

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
app_timer_timeout_handler_t battery_level_meas_timeout_handler(void * p_context) {
    ret_code_t err_code;
    p_bas = (ble_bas_t *) p_context;
    err_code = nrfx_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

/**@brief Update the Inactivity flag of the Features Characteristic
 */
void update_inactivity(void) {
    if(nrf_gpio_pin_read(INT1_AG)) {
        // Enable the Inactivity
        lsm9ds1_configInactivity(&lsm9ds1, LSM9DS1_INACTIVITY_DUR, LSM9DS1_INACTIVITY_THS, false);
    }
    else {
        // Disable the Inactivity
        lsm9ds1_xgWriteByte(&lsm9ds1, ACT_THS, 0);
    }
    features_data.bit.inactivity = nrf_gpio_pin_read(INT2_AG);
}

/**@brief Function for handling the LSM9DS1's measurement timer timeout.
 *
 * @details This function will be called each time the LSM9DS1's measurement timer expires.
 *          This function will start reading the LSM9DS1 data.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
app_timer_timeout_handler_t lsm9ds1_meas_timeout_handler(void * p_context) {
    p_mas = (ble_mas_t *) p_context;
    lsm9ds1_readAccel(&lsm9ds1);
    lsm9ds1_readGyro(&lsm9ds1);
    lsm9ds1_readMag(&lsm9ds1);
    lsm9ds1_readTemp(&lsm9ds1);
    update_inactivity();
    ble_mas_accelerometer_measurement_send(p_mas, lsm9ds1.ax, lsm9ds1.ay, lsm9ds1.az);
    ble_mas_gyroscope_measurement_send(p_mas, lsm9ds1.gx, lsm9ds1.gy, lsm9ds1.gz);
    ble_mas_features_send(p_mas, features_data.byte);
}

/**@brief Monitor Activity Control Point event handler type.
 *        This control point is application specific, you need to implement it
*/
ble_macp_evt_handler_t macp_evt_handler (uint8_t * data, uint8_t size) {
    switch(data[0]) {
        case OP_MACP_CALIBRATE_ALL:
            NRF_LOG_INFO("Calibrating all...");
            NRF_LOG_INFO("Calibrating magnetometer");
            NRF_LOG_DEBUG("Mag bias (current)  x: %d y: %d z: %d", lsm9ds1.mBiasRaw[0], lsm9ds1.mBiasRaw[1], lsm9ds1.mBiasRaw[2]);
            lsm9ds1_calibrateMag(&lsm9ds1, true);
            features_data.bit.calibrated_m = 1;
            NRF_LOG_DEBUG("Mag bias (new)      x: %d y: %d z: %d", lsm9ds1.mBiasRaw[0], lsm9ds1.mBiasRaw[1], lsm9ds1.mBiasRaw[2]);

        case OP_MACP_CALIBRATE_AG:
            NRF_LOG_INFO("Calibrating accelerommeter and gyroscope");
            NRF_LOG_DEBUG("Acc bias (current)  x: %d y: %d z: %d", lsm9ds1.aBiasRaw[0], lsm9ds1.aBiasRaw[1], lsm9ds1.aBiasRaw[2]);
            NRF_LOG_DEBUG("Gyr bias (current)  x: %d y: %d z: %d", lsm9ds1.gBiasRaw[0], lsm9ds1.gBiasRaw[1], lsm9ds1.gBiasRaw[2]);
            lsm9ds1_calibrate(&lsm9ds1, true);
            features_data.bit.calibrated_ag = 1;
            NRF_LOG_DEBUG("Acc bias (new)      x: %d y: %d z: %d", lsm9ds1.aBiasRaw[0], lsm9ds1.aBiasRaw[1], lsm9ds1.aBiasRaw[2]);
            NRF_LOG_DEBUG("Gyr bias (new)      x: %d y: %d z: %d", lsm9ds1.gBiasRaw[0], lsm9ds1.gBiasRaw[1], lsm9ds1.gBiasRaw[2]);
            break;

        case OP_MACP_CALIBRATE_M:
            NRF_LOG_INFO("Calibrating magnetometer");
            NRF_LOG_DEBUG("Mag bias (current)  x: %d y: %d z: %d", lsm9ds1.mBiasRaw[0], lsm9ds1.mBiasRaw[1], lsm9ds1.mBiasRaw[2]);
            lsm9ds1_calibrateMag(&lsm9ds1, true);
            features_data.bit.calibrated_m = 1;
            NRF_LOG_DEBUG("Mag bias (new)      x: %d y: %d z: %d", lsm9ds1.mBiasRaw[0], lsm9ds1.mBiasRaw[1], lsm9ds1.mBiasRaw[2]);
            break;

        default:
            break;
    }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
nrfx_saadc_event_handler_t saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, sizeof(adc_buf)/sizeof(nrf_saadc_value_t));
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(p_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_BUSY) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}


/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t saadc_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    nrfx_saadc_config_t saadc_config = {                                                                               \
    .resolution         = NRF_SAADC_RESOLUTION_10BIT,
    .oversample         = NRF_SAADC_OVERSAMPLE_DISABLED,
    .interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,
    .low_power_mode     = true
    };

    err_code = nrfx_saadc_init(&saadc_config, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(0, &saadc_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(&adc_buf, sizeof(adc_buf)/sizeof(nrf_saadc_value_t));
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for configuring TWIM to do sensor communication.
 */
static void twim_configure(void)
{
    ret_code_t err_code;
    nrfx_twim_config_t nrfx_twim_config = NRFX_TWIM_DEFAULT_CONFIG;

    nrfx_twim_config.scl = SCL_PIN;
    nrfx_twim_config.sda = SDA_PIN;

    err_code = nrfx_twim_init(&nrfx_twim, &nrfx_twim_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrfx_twim_enable(&nrfx_twim);
}

/**@brief Function for configuring the GPIO.
 */
static void gpio_configure(void)
{
    nrf_gpio_cfg_input(INT1_AG, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(INT2_AG, NRF_GPIO_PIN_NOPULL);
}


/**@brief Function for initialize the IMU.
 */
static void imu_init(void)
{
    lsm9ds1_begin(&lsm9ds1, &nrfx_twim, LSM9DS1_AG_ADDR(LSM9DS1_SA0), LSM9DS1_M_ADDR(LSM9DS1_SA1));
    
    // Disable the accelerometer latching interrupt
    lsm9ds1_xgWriteByte(&lsm9ds1, CTRL_REG4, 0x38);

    // Configure accelerometer interrupt
    lsm9ds1_configAccelThs(&lsm9ds1, 80, X_AXIS, 255, false);
    lsm9ds1_configAccelInt(&lsm9ds1, XLIE_XL, false);
    lsm9ds1_configInt(&lsm9ds1, XG_INT1, INT_IG_XL, INT_ACTIVE_HIGH, INT_PUSH_PULL);

    // Configure the inactivity interrupt output pin
    lsm9ds1_configInt(&lsm9ds1, XG_INT2, INT2_INACT, INT_ACTIVE_HIGH, INT_PUSH_PULL);
}

/**@brief Function for initialize the sensor and peripherals.
 */
void sensors_init(void) {
    // Configure peripherals
    adc_configure();
    twim_configure();
    gpio_configure();

    // Init sensors
    imu_init();
}
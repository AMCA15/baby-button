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
#include <nrfx_gpiote.h>
#include <ble.h>
#include <ble_conn_state.h>
#include <ble_bas.h>
#include "ble_mas.h"
#include "SparkFunLSM9DS1.h"
#include <ble_sec.h>

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
#define LSM9DS1_FIFO_THS                16                 /**< Set the LSM9DS1 FIFO Threshold. */

#define INT1_AG      NRF_GPIO_PIN_MAP(1,11)                /**< INT1_AG pin of the LSM9DS1. */
#define INT2_AG      NRF_GPIO_PIN_MAP(1,10)                /**< INT2_AG pin of the LSM9DS1. */
#define INT_M        NRF_GPIO_PIN_MAP(1,13)                /**< INT_M   pin of the LSM9DS1. */
#define DRDY_M       NRF_GPIO_PIN_MAP(1,15)                /**< DRDY_M  pin of the LSM9DS1. */


#define ACC_THS_FOR_SET_PASSKEY  -0.95 / 0.000061          /**< Acceleration threshold for setting the passkey type. */
#define MAG_THS_FOR_SET_PASSKEY      1 / 0.00014           /**< Magnetic field threshold for setting the passkey type. */


#define BURST_BUFFER_SIZE        LSM9DS1_FIFO_THS*6        /**< Buffer size for burst reading on lsm9ds1 registers. */

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

static struct imu_data_t {
    int16_t acc_x[LSM9DS1_FIFO_THS];
    int16_t acc_y[LSM9DS1_FIFO_THS];
    int16_t acc_z[LSM9DS1_FIFO_THS];
    int16_t gyr_x[LSM9DS1_FIFO_THS];
    int16_t gyr_y[LSM9DS1_FIFO_THS];
    int16_t gyr_z[LSM9DS1_FIFO_THS];
} imu_data;

static struct imu_int_t {
    uint8_t is_fifo_full;
    uint8_t is_ig_xl;
    uint8_t is_ig_g;
    uint8_t is_inactive;
} imu_int;

// Feature data Struct
static union features_data_t {
    struct bit_t {
        uint8_t calibrated_ag : 1;
        uint8_t calibrated_m  : 1;
        uint8_t inactivity    : 1;
    } bit;
    uint8_t byte;
} features_data;


static void adc_configure(void);

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
void battery_level_meas_timeout_handler(void * p_context) {
    ret_code_t err_code;
    p_bas = (ble_bas_t *) p_context;
    adc_configure();
    err_code = nrfx_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

/**@brief Update the Inactivity flag of the Features Characteristic
 */
void update_inactivity(void) {
    if(nrf_gpio_pin_read(INT1_AG)) {
        // Enable the Inactivity
        lsm9ds1_configInactivity(&lsm9ds1, LSM9DS1_INACTIVITY_DUR, LSM9DS1_INACTIVITY_THS, false);
        features_data.bit.inactivity = lsm9ds1_getInactivity(&lsm9ds1);
    }
    else {
        // Disable the Inactivity
        lsm9ds1_xgWriteByte(&lsm9ds1, ACT_THS, 0);
    }
}

/**@brief Function for send all the data to all connected clients.
 */
void send_to_all(void) {
    ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_periph_handles();

    for (uint8_t i = 0; i < conn_handles.len; i++)
    {
        // Set the connection handle
        p_mas->conn_handle = conn_handles.conn_handles[i];

        ble_mas_accelerometer_bulk_send(p_mas, &imu_data.acc_x, &imu_data.acc_y, &imu_data.acc_z, LSM9DS1_FIFO_THS);
        ble_mas_gyroscope_bulk_send(p_mas, &imu_data.gyr_x, &imu_data.gyr_y, &imu_data.gyr_z, LSM9DS1_FIFO_THS);
        ble_mas_features_send(p_mas, features_data.byte);
    }
}

/**@brief Set the passkey type.
 * If the thresholds are exceeded, it will set a static passkey, otherwise a random passkey is used
 */
void set_passkey_type(void) {
     if((lsm9ds1.ax <= ACC_THS_FOR_SET_PASSKEY) && (lsm9ds1.mz >= MAG_THS_FOR_SET_PASSKEY)) {
         set_static_passkey();
     }
     else {
         set_random_passkey();
     }
}

// TODO Fix documentation
/**@brief Function for handling the LSM9DS1's measurement timer timeout.
 *
 * @details This function will be called each time the LSM9DS1's measurement timer expires.
 *          This function will start reading the LSM9DS1 data.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
void init_ble_mas_sensor(void * p_context) {
    p_mas = (ble_mas_t *) p_context;
}

/**@brief Function for handling the LSM9DS1's measurement timer timeout.
 *
 * @details This function will be called each time the LSM9DS1's measurement timer expires.
 *          This function will start reading the LSM9DS1 data.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
void acquire_and_send_data(void) {

    uint16_t acc_gyr_buffer[BURST_BUFFER_SIZE];

    if(imu_int.is_fifo_full){
        NRF_LOG_RAW_INFO("Samples in FIFO %d | INT 1 %d | INT 2 %d\n", lsm9ds1_getFIFOSamples(&lsm9ds1), nrf_gpio_pin_read(INT1_AG), nrf_gpio_pin_read(INT2_AG));
        
        lsm9ds1_readMag(&lsm9ds1);
        lsm9ds1_readTemp(&lsm9ds1);

        // TODO Check improvement with this version and the burst reading
        // for(uint8_t i=0; i < LSM9DS1_FIFO_THS; i++) {
        //     lsm9ds1_readAccel(&lsm9ds1);
        //     lsm9ds1_readGyro(&lsm9ds1);
        //     imu_data.acc_x[i] = lsm9ds1.ax;
        //     imu_data.acc_y[i] = lsm9ds1.ay;
        //     imu_data.acc_z[i] = lsm9ds1.az;
        //     imu_data.gyr_x[i] = lsm9ds1.gx;
        //     imu_data.gyr_y[i] = lsm9ds1.gy;
        //     imu_data.gyr_z[i] = lsm9ds1.gz;
        //     // NRF_LOG_RAW_INFO("[%d] ", i);
        //     // NRF_LOG_RAW_INFO("Acc: %d, %d, %d || Gyr: %d, %d, %d\n", imu_data.acc_x[i], imu_data.acc_y[i], imu_data.acc_z[i], imu_data.gyr_x[i], imu_data.gyr_y[i], imu_data.gyr_z[i]);
        // }

        if(features_data.bit.inactivity) {
            lsm9ds1_readAccelGyro_burst(&lsm9ds1, acc_gyr_buffer, LSM9DS1_FIFO_THS, true);
        }
        else {
            lsm9ds1_readAccelGyro_burst(&lsm9ds1, acc_gyr_buffer, LSM9DS1_FIFO_THS, false);
        }
        for(uint8_t i = 0, j = 0; i < LSM9DS1_FIFO_THS; i++, j+=6) {
            imu_data.acc_x[i] = acc_gyr_buffer[j+3];
            imu_data.acc_y[i] = acc_gyr_buffer[j+4];
            imu_data.acc_z[i] = acc_gyr_buffer[j+5];
            imu_data.gyr_x[i] = acc_gyr_buffer[ j ];
            imu_data.gyr_y[i] = acc_gyr_buffer[j+1];
            imu_data.gyr_z[i] = acc_gyr_buffer[j+2];
        }

        NRF_LOG_RAW_INFO("Samples in FIFO %2d | INT 1 %d | INT 2 %d\n", lsm9ds1_getFIFOSamples(&lsm9ds1), nrf_gpio_pin_read(INT1_AG), nrf_gpio_pin_read(INT2_AG));
        
        update_inactivity();
        send_to_all();
        set_passkey_type();
    }
}

/**@brief Monitor Activity Control Point event handler type.
 *        This control point is application specific, you need to implement it
*/
void macp_evt_handler (uint8_t * data, uint8_t size) {
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
void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
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
        nrfx_saadc_uninit();
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
    nrfx_twim_config.interrupt_priority = 6;

    err_code = nrfx_twim_init(&nrfx_twim, &nrfx_twim_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrfx_twim_enable(&nrfx_twim);
}


void gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    switch (pin) {
        case INT2_AG:
            imu_int.is_fifo_full = nrf_gpio_pin_read(INT2_AG);
            acquire_and_send_data();
            break;
    }
}


/**@brief Function for configuring the GPIOTE.
 */
static void gpiote_configure(void)
{
    ret_code_t err_code;
    const nrfx_gpiote_in_config_t nrfx_gpiote_in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

    NRF_LOG_INFO("Configuring GPIOTE");
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
    nrfx_gpiote_in_init(INT1_AG,  &nrfx_gpiote_in_config, gpiote_handler);
    nrfx_gpiote_in_init(INT2_AG,  &nrfx_gpiote_in_config, gpiote_handler);
    nrfx_gpiote_in_event_enable(INT2_AG, true);
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

    // Configure FIFO
    lsm9ds1_setFIFO(&lsm9ds1, FIFO_OFF, 0);
    lsm9ds1_setFIFO(&lsm9ds1, FIFO_CONT, 15);
    lsm9ds1_enableFIFO(&lsm9ds1, true);

    // Configure the inactivity interrupt output pin
    // lsm9ds1_configInt(&lsm9ds1, XG_INT2, INT2_INACT, INT_ACTIVE_HIGH, INT_PUSH_PULL);
    lsm9ds1_configInt(&lsm9ds1, XG_INT2, INT_FTH, INT_ACTIVE_HIGH, INT_PUSH_PULL);
}

static void imu_uninit(void) {
    // Reboot accelerometer and gyroscope memory content
    lsm9ds1_xgWriteByte(&lsm9ds1, CTRL_REG8, 0x01);
    
    // Reboot magnetic sensor memory content
    lsm9ds1_mWriteByte(&lsm9ds1, CTRL_REG2_M, 0x04);
}

void sensors_uninit(void) {
    nrfx_gpiote_uninit();
    imu_uninit();
    nrfx_twim_uninit(&nrfx_twim);
}

void sensors_resume(void) {
    gpiote_configure();
    imu_init();
    // nrfx_twim_enable(&nrfx_twim);
}

/**@brief Function for initialize the sensor and peripherals.
 */
void sensors_init(void) {
    // Configure peripherals
    // adc_configure();
    twim_configure();
    gpiote_configure();

    // Init sensors
    imu_init();
}
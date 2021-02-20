/**
 * @file sensors.h
 * @author Anderson Contreras
 * @brief Function for handling sensors, peripherals and battery stuffs.
 * @version 0.1
 * @date 2020-04-29
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef SENSORS_H__
#define SENSORS_H__

#include <stdint.h>

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
void battery_level_meas_timeout_handler(void * p_context);

/**@brief Function for handling the LSM9DS1's measurement timer timeout.
 *
 * @details This function will be called each time the LSM9DS1's measurement timer expires.
 *          This function will start reading the LSM9DS1 data.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
void lsm9ds1_meas_timeout_handler(void * p_context);

/**@brief Monitor Activity Control Point event handler type. 
 *        This control point is application specific, you need to implement it
*/
void macp_evt_handler (uint8_t * data, uint8_t size);

/**@brief Function for initialize the sensors and peripherals.
 */
void sensors_init(void);

/**@brief Get the fifo status
 * 
 * @return uint8_t 1: The FIFO is full 
 *                 0: The FIFO is empty
 */
uint8_t get_fifo_status(void);

/**@brief Set the fifo status
 * 
 * @param status uint8_t 1: The FIFO is full 
 *                       0: The FIFO is empty
 */
void set_fifo_status(uint8_t status);


#endif // SENSORS_H__
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

#include <app_timer.h>


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
app_timer_timeout_handler_t battery_level_meas_timeout_handler(void * p_context);

/**@brief Function for initialize the sensors and peripherals.
 */
void sensors_init(void);


#endif // SENSORS_H__
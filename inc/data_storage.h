#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include "hal.h"

/**
 * @brief Load configuration data from flash.
 *
 * @details Data loaded are:
 *               - wheels_gap
 *               - ticks_per_m
 *               - angular_trust_threshold
 *               - max_linear_acceleration
 *               - max_angular_acceleration
 *               - cruise_linear_speed
 *               - cruise_angular_speed
 *               - linear_p_coeff
 *               - linear_i_coeff
 *               - linear_d_coeff
 *               - angular_p_coeff
 *               - angular_i_coeff
 *               - angular_d_coeff
 *               - linear_allowance
 *               - angular_allowance
 */
extern void load_data_from_flash(void);

/**
 * @brief Save configuration data in flash.
 *
 * @details Data stored are:
 *               - wheels_gap
 *               - ticks_per_m
 *               - angular_trust_threshold
 *               - max_linear_acceleration
 *               - max_angular_acceleration
 *               - cruise_linear_speed
 *               - cruise_angular_speed
 *               - linear_p_coeff
 *               - linear_i_coeff
 *               - linear_d_coeff
 *               - angular_p_coeff
 *               - angular_i_coeff
 *               - angular_d_coeff
 *               - linear_allowance
 *               - angular_allowance
 *
 * @return An int32_t indicating success, else an error code.
 * @retval FLASH_RETURN_SUCCESS No error.
 * @retval FLASH_RETURN_NO_PERMISSION Access denied.
 * @retval FLASH_RETURN_BAD_FLASH Flash cell error.
 */
extern int32_t store_data_in_flash(void);

#endif /* DATA_STORAGE_H */

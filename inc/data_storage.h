/** @file */

#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include "hal.h"

/**
 * @brief Load configuration data from flash.
 *
 * @details This function fills the "settings" global variable with the values
 *          stored in flash.
 */
extern void load_data_from_flash(void);

/**
 * @brief Save configuration data in flash.
 *
 * @details This function writes in flash the content of the "settings" global
 *          variables.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval FLASH_RETURN_SUCCESS No error.
 * @retval FLASH_RETURN_NO_PERMISSION Access denied.
 * @retval FLASH_RETURN_BAD_FLASH Flash cell error.
 */
extern int32_t store_data_in_flash(void);

#endif /* DATA_STORAGE_H */

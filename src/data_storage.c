/******************************************************************************/
/*                                 Includes                                   */
/******************************************************************************/
#include "data_storage.h"
#include "flash.h"
#include "settings.h"
#include "control.h"

/******************************************************************************/
/*                                Constants                                   */
/******************************************************************************/
#define DATAPAGE_ID 31 /* Id of the flash page used for data storage */

#define DATAPAGE_START 0x0800F800 /* Base address of the non-volatile data storage */

/* Offsets */
#define WHEELS_GAP_OFFSET               0U
#define TICKS_PER_M_OFFSET              1U
#define ANGULAR_TRUST_THRESHOLD_OFFSET  2U
#define MAX_LINEAR_ACCELERATION_OFFSET  3U
#define MAX_ANGULAR_ACCELERATION_OFFSET 4U
#define CRUISE_LINEAR_SPEED_OFFSET      5U
#define CRUISE_ANGULAR_SPEED_OFFSET     6U
#define LINEAR_P_COEFF_OFFSET           7U
#define LINEAR_I_COEFF_OFFSET           8U
#define LINEAR_D_COEFF_OFFSET           9U
#define ANGULAR_P_COEFF_OFFSET          10U
#define ANGULAR_I_COEFF_OFFSET          11U
#define ANGULAR_D_COEFF_OFFSET          12U

#define DATA_NB 13U /* Number of data (uint16_t) stored in flash */

/******************************************************************************/
/*                             Public functions                               */
/******************************************************************************/
void load_data_from_flash(void) {
    uint16_t* ptr = (uint16_t*)DATAPAGE_START;

    wheels_gap               = ptr[WHEELS_GAP_OFFSET];
    ticks_per_m              = ptr[TICKS_PER_M_OFFSET];
    angular_trust_threshold  = ptr[ANGULAR_TRUST_THRESHOLD_OFFSET];
    max_linear_acceleration  = ptr[MAX_LINEAR_ACCELERATION_OFFSET];
    max_angular_acceleration = ptr[MAX_ANGULAR_ACCELERATION_OFFSET];
    cruise_linear_speed      = ptr[CRUISE_LINEAR_SPEED_OFFSET];
    cruise_angular_speed     = ptr[CRUISE_ANGULAR_SPEED_OFFSET];
    linear_p_coeff           = ptr[LINEAR_P_COEFF_OFFSET];
    linear_i_coeff           = ptr[LINEAR_I_COEFF_OFFSET];
    linear_d_coeff           = ptr[LINEAR_D_COEFF_OFFSET];
    angular_p_coeff          = ptr[ANGULAR_P_COEFF_OFFSET];
    angular_i_coeff          = ptr[ANGULAR_I_COEFF_OFFSET];
    angular_d_coeff          = ptr[ANGULAR_D_COEFF_OFFSET];
}

int32_t store_data_in_flash(void) {

    int32_t status;
    uint16_t data[DATA_NB] = {
        wheels_gap,
        ticks_per_m,
        angular_trust_threshold,
        max_linear_acceleration,
        max_angular_acceleration,
        cruise_linear_speed,
        cruise_angular_speed,
        linear_p_coeff,
        linear_i_coeff,
        linear_d_coeff,
        angular_p_coeff,
        angular_i_coeff,
        angular_d_coeff
    };

    status = flashPageErase(DATAPAGE_ID);

    if (FLASH_RETURN_SUCCESS == status) {
        status = flashWrite(DATAPAGE_START, (const char *)data, DATA_NB * sizeof(uint16_t));
    }

    return status;
}

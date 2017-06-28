/******************************************************************************/
/*                                 Includes                                   */
/******************************************************************************/
#include "data_storage.h"
#include "flash.h"
#include "settings.h"

/******************************************************************************/
/*                                Constants                                   */
/******************************************************************************/
#define DATAPAGE_ID 31 /* Id of the flash page used for data storage */

#define DATAPAGE_START 0x0800F800U /* Base address of the non-volatile data storage */

/******************************************************************************/
/*                             Public functions                               */
/******************************************************************************/
void load_data_from_flash(void) {
    settings = *(robot_settings_t*)DATAPAGE_START;
}

int32_t store_data_in_flash(void) {
    int32_t status;

    status = flashPageErase(DATAPAGE_ID);

    if (FLASH_RETURN_SUCCESS == status) {
        status = flashWrite(DATAPAGE_START, (const char *)&settings, sizeof(robot_settings_t));
    }

    return status;
}

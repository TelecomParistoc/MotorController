#ifndef POSITION_H
#define POSITION_H

#include "hal.h"

extern int32_t set_heading(int16_t heading);
extern int32_t set_pitch(int16_t pitch);
extern int32_t set_roll(int16_t roll);

/**
 * @brief Set the euler heading angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @param[in] heading The current heading angle.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error, offset set.
 * @retval INVALID_PARAMETER heading out of range.
 */
extern int16_t get_direction(void);

/**
 * @brief Set the euler pitch angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error, offset set.
 * @retval INVALID_PARAMETER pitch out of range.
 *
 * @param[in] pitch The current pitch angle.
 */
extern int16_t get_relative_pitch(void);

/**
 * @brief Set the euler roll angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error, offset set.
 * @retval INVALID_PARAMETER roll out of range.
 *
 * @param[in] roll The current roll angle.
 */
extern int16_t get_relative_roll(void);
#endif

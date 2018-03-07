/** @file */

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include "hal.h"

#define HEADING_MAX_VALUE 5760
#define HEADING_MIN_VALUE 0
#define HEADING_RANGE 5760

/******************************************************************************/
/*                            Public variables                                */
/******************************************************************************/

/**
 * The last computed orientation of the robot.
 */
extern int16_t orientation;
extern float coding_wheels_orientation;
extern int16_t IMU_orientation;

extern float delta_alpha;

/**
 * Offset to apply to the raw value given by the IMU.
 */
extern int16_t heading_offset;

/******************************************************************************/
/*                         Function prototypes                                */
/******************************************************************************/

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
extern int32_t set_orientation(int16_t heading);

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
extern int32_t set_pitch(int16_t pitch);

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
extern int32_t set_roll(int16_t roll);

/**
 * @brief Get the relative heading (relative to the last setting) in trigo sense.
 *
 * @return The relative heading. The range is [0, 5760] and the value decreases
 *         when turning clockwise.
 *         The sense of the angle is changed compared to the value returned by
 *         the BNO055.
 */
extern int16_t get_relative_heading(void);

/**
 * @brief Get the relative pitch angle (relative to the last setting).
 *
 * @return The relative pitch. The range is [-2880, 2880] and the value increases
 *         when the inclination increases.
 */
extern int16_t get_relative_pitch(void);

/**
 * @brief Get the relative roll angle (relative to the last setting).
 *
 * @return The relative roll. The range is [-1440, 1440] and the value increases
 *         when the inclination increases.
 */
extern int16_t get_relative_roll(void);

/**
 * @brief Update the current orientation.
 *
 * @details This function uses either the IMU or the coding wheels to compute
 *          the new orientation value. A threshold on the angular speed is used
 *          to select one of the two sources.
 */
extern void update_orientation(void);

#endif /* ORIENTATION_H */

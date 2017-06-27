#ifndef POSITION_H
#define POSITION_H

#include "ch.h"

/**
 * For tests only.
 */
extern int32_t previous_left_ticks;
extern int32_t previous_right_ticks;

/**
 * Variation of the left coding wheels, in ticks.
 */
extern int32_t delta_left;

/**
 * Variation of the right coding wheels, in ticks.
 */
extern int32_t delta_right;

/**
 * Current x coordinate of the robot center, in mm.
 */
extern int32_t current_x;

/**
 * Current y coordinate of the robot center, in mm.
 */
extern int32_t current_y;

/**
 * @brief Compute the coding wheels movements.
 */
extern void compute_movement(void);

/**
 * @brief Update the position according to the information given by the coding wheels.
 *
 * @details This function must be called AFTER update_orientation(), as it
 *          considers that the 'orientation' variable holds the new orientation.
 */
extern void update_position(void);

#endif /* POSITION_H */

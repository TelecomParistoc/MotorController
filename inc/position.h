#ifndef POSITION_H
#define POSITION_H

#include "ch.h"

extern int32_t previous_left_ticks;
extern int32_t previous_right_ticks;

extern int32_t delta_left;
extern int32_t delta_right;

extern int32_t current_x;
extern int32_t current_y;

extern void compute_movement(void);

/**
 * Must be called AFTER update_orientation().
 */
extern void update_position(void);

#endif /* POSITION_H */

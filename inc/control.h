#ifndef CONTROL_H
#define CONTROL_H

#include "hal.h"

#define CONTROL_STACK_SIZE 1024

extern volatile int16_t goal_mean_dist;
extern volatile uint16_t goal_heading;
extern volatile int16_t heading_dist_sync_ref;

extern volatile uint16_t linear_p_coeff;
extern volatile uint16_t linear_i_coeff;
extern volatile uint16_t linear_d_coeff;
extern volatile uint16_t angular_p_coeff;
extern volatile uint16_t angular_i_coeff;
extern volatile uint16_t angular_d_coeff;
extern THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);

extern THD_FUNCTION(control_thread, p);

#endif /* CONTROL_H */

#ifndef CONTROL_H
#define CONTROL_H

#include "hal.h"

#define CONTROL_STACK_SIZE 1024
#define INT_POS_STACK_SIZE 1024

extern volatile int16_t goal_mean_dist;
extern volatile uint16_t goal_heading;
extern volatile int16_t heading_dist_sync_ref;

extern volatile bool dist_command_received;
extern volatile bool heading_command_received;

extern volatile int32_t current_distance;

extern volatile uint16_t linear_p_coeff;
extern volatile uint16_t linear_i_coeff;
extern volatile uint16_t linear_d_coeff;

extern volatile uint16_t angular_p_coeff;
extern volatile uint16_t angular_i_coeff;
extern volatile uint16_t angular_d_coeff;

extern volatile uint8_t master_stop;

extern THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);
extern THD_WORKING_AREA(wa_int_pos, INT_POS_STACK_SIZE);

extern THD_FUNCTION(control_thread, p);
extern THD_FUNCTION(int_pos_thread, p);


#endif /* CONTROL_H */

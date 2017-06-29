#ifndef CONTROL_H
#define CONTROL_H

#include "hal.h"

#define CONTROL_STACK_SIZE 1024
#define INT_POS_STACK_SIZE 1024

/**
 * Distance to travel (command received from master), in mm.
 */
extern volatile int32_t goal_mean_dist;

/**
 * Heading to reach (command received from the master), in range [0, 5760].
 */
extern volatile uint16_t goal_heading;

/**
 * Distance from which rotation can start (if linear and angular movements are
 * performed at the same time), in mm.
 */
extern volatile int16_t heading_dist_sync_ref;

/**
 * Boolean value indicating whether a new (linear) command has been received.
 */
extern volatile bool dist_command_received;

/**
 * Distance currently travelled since last (linear) command has been received, in mm.
 */
extern volatile int32_t current_distance;

/**
 * PID coeffs for linear movements.
 */
extern volatile uint16_t linear_p_coeff;
extern volatile uint16_t linear_i_coeff;
extern volatile uint16_t linear_d_coeff;

/**
 * PID coeffs for angular movements.
 */
extern volatile uint16_t angular_p_coeff;
extern volatile uint16_t angular_i_coeff;
extern volatile uint16_t angular_d_coeff;

/**
 * Boolean value indicating that motors must be stopped.
 */
extern volatile uint8_t master_stop;

extern binary_semaphore_t reset_pos_sem;
extern volatile int8_t reset_pos_direction;
extern volatile int8_t reset_pos_orientation;

extern THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);
extern THD_WORKING_AREA(wa_int_pos, INT_POS_STACK_SIZE);
extern THD_WORKING_AREA(wa_reset_pos, INT_POS_STACK_SIZE);

extern THD_FUNCTION(control_thread, p);
extern THD_FUNCTION(int_pos_thread, p);
extern THD_FUNCTION(reset_pos_thread, p);


#endif /* CONTROL_H */

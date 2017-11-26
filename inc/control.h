#ifndef CONTROL_H
#define CONTROL_H

/******************************************************************************/
/*                               Includes                                     */
/******************************************************************************/
#include "hal.h"

/******************************************************************************/
/*                               Constants                                    */
/******************************************************************************/
#define CONTROL_STACK_SIZE 1024
#define INT_POS_STACK_SIZE 1024

/******************************************************************************/
/*                                Types                                       */
/******************************************************************************/
typedef struct {
    int32_t mean_dist; /* Distance to travel (command received from master), in mm */
    uint16_t heading; /* Heading to reach (command received from master), in range [0, 5760] */
    int16_t heading_dist_sync_ref; /* Distance from which rotation can start (if linear and angular movements are performed at the same time), in mm */
} goal_t;

/******************************************************************************/
/*                              Variables                                     */
/******************************************************************************/
/**
 * Goal values received from master.
 */
extern volatile goal_t goal;

/**
 * Boolean value indicating whether a new (linear) command has been received.
 */
extern volatile bool dist_command_received;

/**
 * Distance currently travelled since last (linear) command has been received, in mm.
 */
extern volatile int32_t current_distance;

/**
 * Boolean value indicating that motors must be stopped.
 */
extern volatile uint8_t master_stop;

/**
 * Boolean value indicating whether current translation is finished.
 */
extern volatile bool translation_ended;

/**
 * Boolean value indicating whether current rotation is finished.
 */
extern volatile bool rotation_ended;

extern binary_semaphore_t reset_orientation_sem;
extern volatile int8_t reset_orientation_direction;
extern volatile int16_t reset_orientation_orientation;

extern THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);
extern THD_WORKING_AREA(wa_int_pos, INT_POS_STACK_SIZE);
extern THD_WORKING_AREA(wa_reset_pos, INT_POS_STACK_SIZE);

extern THD_FUNCTION(control_thread, p);
extern THD_FUNCTION(int_pos_thread, p);
extern THD_FUNCTION(reset_pos_thread, p);


/******************************************************************************/
/*                          Local Functions                                   */
/******************************************************************************/

float compute_target(float, float, float, float, float);

#endif /* CONTROL_H */

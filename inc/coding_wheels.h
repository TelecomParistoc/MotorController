#ifndef CODING_WHEELS_H
#define CODING_WHEELS_H

#include "hal.h"

typedef enum {
    DIRECT = 0U,
    INDIRECT = 1U
} wheel_orientation_t;

/*
 * The `orientation` parameters allow the user to specify which rotation direction
 * corresponds to a forward movement of the robot.
 */
typedef struct {
    uint32_t initial_right_ticks;
    wheel_orientation_t right_wheel_orientation;
    uint32_t initial_left_ticks;
    wheel_orientation_t left_wheel_orientation;
} coding_wheels_config_t;

/*
 * Number of ticks counted by the right coding wheel.
 */
extern volatile uint32_t right_ticks;

/*
 * Number of ticks counted by the left coding wheel.
 */
extern volatile uint32_t left_ticks;

/*
 * @brief Perform all the initializations required by the coding wheels.
 *
 * @param[in] config The initial configuration of the coding wheels driver.
 *            For common use, initial_left_ticks and initial_right_ticks should
 *            be set to a not too small value because if the coding wheel turns
 *            backwards, the counter will decrease.
 */
extern void init_coding_wheels(coding_wheels_config_t config);

#endif /* CODING_WHEELS_H */

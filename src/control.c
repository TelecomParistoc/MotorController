#include "control.h"
#include "coding_wheels.h"

volatile uint16_t linear_p_coeff;
volatile uint16_t linear_i_coeff;
volatile uint16_t linear_d_coeff;
volatile uint16_t angular_p_coeff;
volatile uint16_t angular_i_coeff;
volatile uint16_t angular_d_coeff;

THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);

extern uint32_t goal_mean_distance;
extern int32_t linear_p_coeff;
extern int32_t linear_i_coeff;
extern int32_t linear_d_coeff;
extern int32_t max_delta_pwm_command; /* max_linear_acceleration * SAMPLING_PERIOD / alpha with v = alpha * PWM */

#define SAMPLING_PERIOD 10 /* in ms */

extern THD_FUNCTION(control_thread, p) {
    (void)p;
    uint32_t current_distance;
    int32_t prev_epsilon;
    int32_t epsilon;
    int32_t epsilon_sum;
    int32_t linear_p;
    int32_t linear_i;
    int32_t linear_d;
    uint32_t command[2];
    uint32_t prev_command[2];

    while (TRUE) {
        /* Compute epsilon and related input values */
        prev_epsilon = epsilon;
        current_distance = 10 * (left_ticks + right_ticks) / (2 * ticks_per_cm); /* In mm */
        epsilon = goal_mean_distance - current_distance;
        epsilon_sum += epsilon;

        /* Linear PID */
        linear_p = linear_p_coeff * epsilon;

        linear_i = linear_i_coeff * epsilon_sum;

        linear_d = (epsilon - prev_epsilon) * linear_d_coeff;

        linear_command = linear_p + linear_i + linear_d;

        /* Angular PID */

        /* Motor command */
        prev_command[MOTOR_LEFT] = command[MOTOR_LEFT];
        command[MOTOR_LEFT] = linear_command;

        if (command[MOTOR_LEFT] - prev_command[MOTOR_LEFT] > max_delta_pwm_command) {
            command[MOTOR_LEFT] = prev_command[MOTOR_LEFT] + max_delta_pwm_command;
        }

        if (command[MOTOR_LEFT] > max_command) {
            command[MOTOR_LEFT] = max_command;
        }

        if (command[MOTOR_LEFT] - prev_command[MOTOR_LEFT] < -max_delta_pwm_command) {
            command[MOTOR_LEFT] = prev_command[MOTOR_LEFT] - max_delta_pwm_command;
        }

        if (command[MOTOR_LEFT] < min_command) {
            command[MOTOR_LEFT] = min_command;
        }

        prev_command[MOTOR_RIGHT] = command[MOTOR_RIGHT];
        command[MOTOR_RIGHT] = linear_command;

        if (command[MOTOR_RIGHT] - prev_command[MOTOR_RIGHT] > max_delta_pwm_command) {
            command[MOTOR_RIGHT] = prev_command[MOTOR_RIGHT] + max_delta_pwm_command;
        }

        if (command[MOTOR_RIGHT] > max_command) {
            command[MOTOR_RIGHT] = max_command;
        }

        if (command[MOTOR_RIGHT] - prev_command[MOTOR_RIGHT] < -max_delta_pwm_command) {
            command[MOTOR_RIGHT] = prev_command[MOTOR_RIGHT] - max_delta_pwm_command;
        }

        if (command[MOTOR_RIGHT] < min_command) {
            command[MOTOR_RIGHT] = min_command;
        }


        /* Apply new commands */
        for (motor_t motor = 0; motor < 1; ++i) {
            /* Change direction if required */
            if (((command[motor] < 0) && (prev_command[motor] >= 0))
                || ((command[motor] >= 0) && (prev_command[motor] < 0))) {
                    toggle_direction(motor);
            }

            /* Set new speed */
            if (command[motor] < 0) {
                set_speed(motor, -command);
            } else {
                set_speed(motor, command);
            }
        }

        chThdSleepMilliseconds(CONTROL_PERIOD);
    }
}

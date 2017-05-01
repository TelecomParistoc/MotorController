#include "control.h"
#include "coding_wheels.h"
#include "motor.h"
#include "settings.h"
#include "orientation.h"

/* Target values */
volatile uint16_t goal_mean_dist;
volatile uint16_t goal_heading;
volatile uint16_t heading_dist_sync_ref;

/* Linear PID coeffs */
volatile uint16_t linear_p_coeff;
volatile uint16_t linear_i_coeff;
volatile uint16_t linear_d_coeff;

/* Angular PID coeffs */
volatile uint16_t angular_p_coeff;
volatile uint16_t angular_i_coeff;
volatile uint16_t angular_d_coeff;

#define REDUCTION_FACTOR 1000

THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);

int32_t max_linear_delta_pwm_command; /* max_linear_acceleration * CONTROL_PERIOD / alpha with v = alpha * PWM */
int32_t max_angular_delta_pwm_command;

#define CONTROL_PERIOD 10 /* in ms */
#define ALPHA 2

extern THD_FUNCTION(control_thread, p) {
    (void)p;
    uint32_t current_distance;

    /* Linear values */
    int32_t prev_linear_epsilon;
    int32_t linear_epsilon;
    int32_t linear_epsilon_sum;

    /* Angular values */
    int32_t prev_angular_epsilon;
    int32_t angular_epsilon;
    int32_t angular_epsilon_sum;

    /* Linear PID  */
    int32_t linear_p;
    int32_t linear_i;
    int32_t linear_d;

    /* Angular PID */
    int32_t angular_p;
    int32_t angular_i;
    int32_t angular_d;

    /* Previous target distance, used to know whether a new target has been received */
    uint16_t prev_goal_dist;

    /* Commands for motors and other related local variables */
    int32_t prev_command[2];
    int32_t command[2];
    int32_t prev_linear_command;
    int32_t linear_command;
    int32_t prev_angular_command;
    int32_t angular_command;
    int32_t tmp_command;

    /* Initialise the variables */
    prev_goal_dist = goal_mean_dist;

    linear_epsilon_sum = 0;
    prev_linear_epsilon = 0;
    prev_linear_command = 0;

    angular_epsilon_sum = 0;
    prev_angular_epsilon = 0;
    prev_angular_command = 0;

    prev_command[MOTOR_LEFT] = 0;
    prev_command[MOTOR_RIGHT] = 0;

    while (TRUE) {

        /* Reset the PID sums if a new target has been received */
        if (prev_goal_dist != goal_mean_dist) {
            prev_goal_dist = goal_mean_dist;
            linear_epsilon_sum = 0;
            angular_epsilon_sum = 0;
        }

        /* Compute the settings value, in case max accelerations have changed */
        max_linear_delta_pwm_command = max_linear_acceleration * CONTROL_PERIOD / ALPHA;
        max_angular_delta_pwm_command = max_angular_acceleration * CONTROL_PERIOD / ALPHA;

        /* Compute linear_epsilon and related input values */
        prev_linear_epsilon = linear_epsilon;
        current_distance = 10 * (left_ticks + right_ticks) / (2 * ticks_per_cm); /* In mm */
        linear_epsilon = goal_mean_dist - current_distance;
        linear_epsilon_sum += linear_epsilon;

        /* Linear PID */
        linear_p = linear_p_coeff * linear_epsilon;

        linear_i = linear_i_coeff * linear_epsilon_sum;

        linear_d = linear_d_coeff * (linear_epsilon - prev_linear_epsilon);

        prev_linear_command = linear_command;
        linear_command = (linear_p + linear_i + linear_d) / REDUCTION_FACTOR;

        /* Limit linear acceleration/deceleration */
        if ((int32_t)(linear_command - prev_linear_command) > max_linear_delta_pwm_command) {
            linear_command = prev_linear_command + max_linear_delta_pwm_command;
        } else if ((int32_t)(linear_command - prev_linear_command) < -max_linear_delta_pwm_command) {
            linear_command = prev_linear_command - max_linear_delta_pwm_command;
        }

        if (current_distance >= heading_dist_sync_ref) {
            /* Compute angular_epsilon and related input values */
            prev_angular_epsilon = angular_epsilon;
            angular_epsilon = goal_heading - orientation;
            angular_epsilon_sum += angular_epsilon;

            /* Angular PID */
            angular_p = angular_p_coeff * angular_epsilon;

            angular_i = angular_i_coeff * angular_epsilon_sum;

            angular_d = angular_d_coeff * (angular_epsilon - prev_angular_epsilon);

            prev_angular_command = angular_command;
            angular_command = (angular_p + angular_i + angular_d) / REDUCTION_FACTOR;

            /* Limit angular acceleration/deceleration */
            if ((int32_t)(angular_command - prev_angular_command) > max_angular_delta_pwm_command) {
                angular_command = prev_angular_command + max_angular_delta_pwm_command;
            } else if ((int32_t)(angular_command - prev_angular_command) < -max_angular_delta_pwm_command) {
                angular_command = prev_angular_command - max_angular_delta_pwm_command;
            }
        } else {
            prev_angular_command = 0;
            angular_command = 0;
        }

        /* Motor commands */
        /* If left wheel required speed is too high, reduce both components */
        tmp_command = linear_command + angular_command;
        if ((tmp_command > MAX_COMMAND) || (tmp_command < -MAX_COMMAND)) {
            linear_command *= tmp_command;
            linear_command /= MAX_COMMAND;

            angular_command *= tmp_command;
            angular_command /= MAX_COMMAND;

        }

        /* If right wheel required speed is too high, reduce both components */
        tmp_command = linear_command - angular_command;
        if ((tmp_command > MAX_COMMAND) || (tmp_command < -MAX_COMMAND)) {
            linear_command *= tmp_command;
            linear_command /= MAX_COMMAND;

            angular_command *= tmp_command;
            angular_command /= MAX_COMMAND;

        }

        /* Compute new commands */
        prev_command[MOTOR_LEFT] = command[MOTOR_LEFT];
        command[MOTOR_LEFT] = linear_command + angular_command;

        prev_command[MOTOR_RIGHT] = command[MOTOR_RIGHT];
        command[MOTOR_RIGHT] = linear_command - angular_command;

        /* Apply new commands */
        motor_t motor;
        for (motor = MOTOR_LEFT; motor < MOTOR_RIGHT; ++motor) {
            /* Change direction if required */
            if (((command[motor] < 0) && (prev_command[motor] >= 0))
                || ((command[motor] >= 0) && (prev_command[motor] < 0))) {
                    motor_toggle_direction(motor);
            }

            /* Set new speed */
            if (command[motor] < 0) {
                motor_set_speed(motor, -command[motor]);
            } else {
                motor_set_speed(motor, command[motor]);
            }
        }

        /* Sleep until next period */
        chThdSleepMilliseconds(CONTROL_PERIOD);
    }
}

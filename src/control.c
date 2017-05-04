#include "control.h"
#include "coding_wheels.h"
#include "motor.h"
#include "settings.h"
#include "orientation.h"
#include "position.h"
#include "RTT/SEGGER_RTT.h"

#define MAX_PWM 30

#define ABS(x) ((x > 0) ? x : -x)

/* Target values */
volatile int16_t goal_mean_dist;
volatile uint16_t goal_heading;
volatile int16_t heading_dist_sync_ref;

/* Linear PID coeffs */
volatile uint16_t linear_p_coeff;
volatile uint16_t linear_i_coeff;
volatile uint16_t linear_d_coeff;

/* Angular PID coeffs */
volatile uint16_t angular_p_coeff;
volatile uint16_t angular_i_coeff;
volatile uint16_t angular_d_coeff;

#define REDUCTION_FACTOR_P 1000
#define REDUCTION_FACTOR_I 10000
#define REDUCTION_FACTOR_D 10000

THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);

int32_t max_linear_delta_pwm_command; /* max_linear_acceleration * CONTROL_PERIOD / alpha with v = alpha * PWM */
int32_t max_angular_delta_pwm_command;

#define CONTROL_PERIOD 50 /* in ms */
#define ALPHA 80

extern THD_FUNCTION(control_thread, p) {
    (void)p;
    int32_t current_distance;

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

    int32_t saved_left_ticks;
    int32_t saved_right_ticks;

    /* Previous target distance, used to know whether a new target has been received */
    int16_t prev_goal_dist;

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
    linear_command = 0;

    angular_epsilon_sum = 0;
    prev_angular_epsilon = 0;
    prev_angular_command = 0;
    angular_command = 0;

    prev_command[MOTOR_LEFT] = 0;
    prev_command[MOTOR_RIGHT] = 0;

    while (TRUE) {

        compute_movement();
        update_orientation();

        /* Reset the PID sums if a new target has been received */
        if (prev_goal_dist != goal_mean_dist) {
            prev_goal_dist = goal_mean_dist;
            linear_epsilon_sum = 0;
            angular_epsilon_sum = 0;
            saved_left_ticks = left_ticks;
            saved_right_ticks = right_ticks;
        }

        /* Compute the settings value, in case max accelerations have changed */
        max_linear_delta_pwm_command = max_linear_acceleration * CONTROL_PERIOD / ALPHA;
        max_angular_delta_pwm_command = max_angular_acceleration * CONTROL_PERIOD / ALPHA;

        /* Compute linear_epsilon and related input values */
        prev_linear_epsilon = linear_epsilon;
        current_distance = 10 * ((left_ticks - saved_left_ticks) + (right_ticks - saved_right_ticks)) / (2 * ticks_per_cm); /* In mm */
        linear_epsilon = goal_mean_dist - current_distance;
        linear_epsilon_sum += linear_epsilon;

        /* Linear PID */
        linear_p = (linear_p_coeff * linear_epsilon) / REDUCTION_FACTOR_P;

        linear_i = (linear_i_coeff * linear_epsilon_sum) / REDUCTION_FACTOR_I;

        linear_d = (linear_d_coeff * (linear_epsilon - prev_linear_epsilon)) / REDUCTION_FACTOR_D;

        prev_linear_command = linear_command;
        linear_command = linear_p + linear_i + linear_d;
        printf("linear %d (%d, %d, %d)\r\n", linear_command, linear_p, linear_i, linear_p);

        /* Limit linear acceleration/deceleration */
        if ((int32_t)(linear_command - prev_linear_command) > max_linear_delta_pwm_command) {
            linear_command = prev_linear_command + max_linear_delta_pwm_command;
        } else if ((int32_t)(linear_command - prev_linear_command) < -max_linear_delta_pwm_command) {
            linear_command = prev_linear_command - max_linear_delta_pwm_command;
        }

        if (ABS(current_distance) >= heading_dist_sync_ref) {
            /* Compute angular_epsilon and related input values */
            prev_angular_epsilon = angular_epsilon;
            angular_epsilon = goal_heading - orientation;
            angular_epsilon_sum += angular_epsilon;

            /* Angular PID */
            angular_p = (angular_p_coeff * angular_epsilon) / REDUCTION_FACTOR_P;

            angular_i = (angular_i_coeff * angular_epsilon_sum) / REDUCTION_FACTOR_I;

            angular_d = (angular_d_coeff * (angular_epsilon - prev_angular_epsilon)) / REDUCTION_FACTOR_D;

            prev_angular_command = angular_command;
            angular_command = angular_p + angular_i + angular_d;
            printf("angular %d \r\n", angular_command);

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
        tmp_command = ABS(linear_command + angular_command);
        if (tmp_command > MAX_PWM) {
            linear_command *= MAX_PWM;
            linear_command /= tmp_command;

            angular_command *= MAX_PWM;
            angular_command /= tmp_command;
        }

        /* If right wheel required speed is too high, reduce both components */
        tmp_command = ABS(linear_command - angular_command);
        if (tmp_command > MAX_PWM) {
            linear_command *= MAX_PWM;
            linear_command /= tmp_command;

            angular_command *= MAX_PWM;
            angular_command /= tmp_command;
        }

        printf("lin %d\r\n", linear_command);
        /* Compute new commands */
        prev_command[MOTOR_LEFT] = command[MOTOR_LEFT];
        command[MOTOR_LEFT] = linear_command + angular_command;
        if ((command[MOTOR_LEFT] < MIN_COMMAND) && (command[MOTOR_LEFT] > 0)) {
            command[MOTOR_LEFT] = MIN_COMMAND;
        } else if ((command[MOTOR_LEFT] > -MIN_COMMAND) && (command[MOTOR_LEFT] < 0)) {
            command[MOTOR_LEFT] = -MIN_COMMAND;
        }

        prev_command[MOTOR_RIGHT] = command[MOTOR_RIGHT];
        command[MOTOR_RIGHT] = linear_command - angular_command;
        if ((command[MOTOR_RIGHT] < MIN_COMMAND) && (command[MOTOR_RIGHT] > 0)) {
            command[MOTOR_RIGHT] = MIN_COMMAND;
        } else if ((command[MOTOR_RIGHT] > -MIN_COMMAND) && (command[MOTOR_RIGHT] < 0)) {
            command[MOTOR_RIGHT] = -MIN_COMMAND;
        }

        /* Apply new commands */
        motor_t motor;
        for (motor = MOTOR_LEFT; motor <= MOTOR_RIGHT; ++motor) {
            /* Change direction if required */
            if (((command[motor] < 0) && (prev_command[motor] >= 0))
                || ((command[motor] >= 0) && (prev_command[motor] < 0))) {
                    motor_toggle_direction(motor);
                    printf("toggle %d (%d, %d)\r\n", motor, command[motor], prev_command[motor]);
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

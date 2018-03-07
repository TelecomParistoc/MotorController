#include "control.h"
#include "coding_wheels.h"
#include "motor.h"
#include "settings.h"
#include "orientation.h"
#include "position.h"
#include "log.h"
#include "math.h"

/******************************************************************************/
/*                              Local macros                                  */
/******************************************************************************/
/* Maximum command value allowed (absolute maximum is 100) */
#define MAX_PWM 70

/* Reduction factors for the PID coeffs */
#define REDUCTION_FACTOR_P 1000
#define REDUCTION_FACTOR_I 10000
#define REDUCTION_FACTOR_D 10000

/* Period of the int_pos thread, in ms */
#define INT_POS_PERIOD 10

/* Period of current position update, in ms */
#define POS_PERIOD 100

/* Period of the control thread, in ms */
#define CONTROL_PERIOD 1

#define RESET_PERIOD 300

/* Returns the absolute value of the parameter */
#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Returns the sign of the parameter (1 or -1) */
#define SIGN(x) (((x) < 0) ? (-1) : 1)

/* Selects the "urgent stop" strategy */
#define BASIC_STOP 0

/* constants used to return against the wall */
/* minimum distance to travel before looking at the stop condition, in mm*/
#define MIN_MOVE_BEFORE_STOP 3
/* maximum time allowed to return against the wall, in ms */
#define MAX_DELAY_TO_RETURN 5000

/*
 * For all variables: goal refers to an instruction sent by the master, target
 * to an intermediate objective.
 */
/******************************************************************************/
/*                            Public variables                                */
/******************************************************************************/
volatile goal_t goal;

volatile bool dist_command_received;

/* Current distance travelled since last goal_mean_dist update */
volatile int32_t current_distance;

/* Boolean value to stop motors whatever the commands are */
volatile uint8_t master_stop;

/* Boolean indicating whether a new distance command has been received from the
   master. It's set to TRUE in i2c_interface. */
volatile bool dist_command_updated;

volatile bool translation_ended;
volatile bool rotation_ended;

BSEMAPHORE_DECL(reset_orientation_sem, TRUE);
volatile int8_t reset_orientation_direction;
volatile int16_t reset_orientation_orientation;

/******************************************************************************/
/*                             Local types                                    */
/******************************************************************************/
typedef struct {
    int32_t p;
    int32_t i;
    int32_t d;
} pid_t;

typedef struct {
    int32_t prev_epsilon;
    int32_t epsilon;
    int32_t epsilon_sum;
} control_values_t;

/******************************************************************************/
/*                          Local variables                                   */
/******************************************************************************/
/* Intermediate value computed by the int_pos thread */
static volatile int32_t target_dist;
static volatile uint16_t target_heading;

static volatile bool orientation_control = TRUE;

/* Threads stacks */
THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);
THD_WORKING_AREA(wa_int_pos, INT_POS_STACK_SIZE);
THD_WORKING_AREA(wa_reset_pos, INT_POS_STACK_SIZE);


/******************************************************************************/
/*                          Local Functions                                   */
/******************************************************************************/

/*
parameters :
  t                 : time, in seconds. t = 0 is the begginning of the move
                      must be positive
  inc_acceleration  : acceleration when the speed of the robot goes increases
                      in m / s^2
  dec_acceleration  : acceleration when the speed of the robot goes decreases
                      in m / s^2
  cruising_speed    : maximum speed of the robot
                      in m / s
  final_x           : final destination of the robot, in m

  NOTE :
    only the signs of t and final_x matter
    the appropriate signs for inc_acceleration, dec_acceleration, cruising_speed
    are calculated

returns  the intermediate target to reach at time t

speed graph :
          ----------------------
        / ^                     ^\
       /  |                     | \
------/   |                     |  \--------
      ^   ^                     ^  ^
      |   |                     |  |
      |   |                     |  |
    t=0  t=t1                 t=t3 t=t4

*/
float compute_target(float t, float inc_acceleration, float dec_acceleration,
                   float cruising_speed, float final_x)
{

    /*  signs computation */
    if (final_x >= 0) {
      inc_acceleration  =   ABS(inc_acceleration);
      dec_acceleration  = - ABS(dec_acceleration);
      cruising_speed    =   ABS(cruising_speed);
    }
    else {
      inc_acceleration  = - ABS(inc_acceleration);
      dec_acceleration  =   ABS(dec_acceleration);
      cruising_speed    = - ABS(cruising_speed);
    }

    /* t1 and t2 computation, see graph above */
    float t1 = cruising_speed / inc_acceleration;
    float t2 = final_x / cruising_speed + cruising_speed / 2 * (1 / inc_acceleration + 1 / dec_acceleration);

    /* if cruising_speed is never reached */
    if (t2 <= t1){
        float t4_square = 2 * final_x / (inc_acceleration * (1 - inc_acceleration / dec_acceleration));

        if (t * t <= t4_square) return inc_acceleration * t * t / 2;

        float t4_inv = 2 / sqrt(t4_square);
        float delta_t = t - t4_inv * final_x / inc_acceleration;

        if (delta_t >= 0) return final_x;
        return dec_acceleration / 2 * delta_t * delta_t + final_x;
    }

    float t3 = t2 - cruising_speed /dec_acceleration;

    /* the result is computed in function of the part of the graph at which t belongs */
    if (t < 0) return 0;
    else if (t <= t1 && t <= t2) return inc_acceleration * t * t / 2;
    else if (t <= t2) return t1 * cruising_speed / 2 + cruising_speed * (t - t1);
    else if (t <= t3)
      return final_x + cruising_speed * cruising_speed / 2 / dec_acceleration \
              + (t - t2) * (cruising_speed + dec_acceleration / 2 * (t - t2));
    else return final_x;
}

/******************************************************************************/
/*                         Public functions                                   */
/******************************************************************************/



extern THD_FUNCTION(int_pos_thread, p) {
    (void)p;

    /*
     * All times are in seconds.
     * All distances are in millimeters.
     */

    /* Time */
    float linear_t = 0.0; /* in s */
    float angular_t = 0.0; /* in s */

    int16_t prev_goal_heading = 0;
    int16_t delta_heading = 0;
    int16_t initial_heading = 0;
    int16_t tmp_target_heading = 0;
    uint32_t update_position_counter = 0U;

    uint32_t start_time;

    while (TRUE) {
        start_time = chVTGetSystemTime();

        /* Acquire sensors data and update localisation */
        compute_movement();
        update_orientation();
        update_position_counter++;
        //TODO : est-ce qu'on ne peut pas recalculer a chaque fois ?
        if (update_position_counter > (uint32_t)(POS_PERIOD / INT_POS_PERIOD)) {
            update_position();
            update_position_counter = 0U;
        }

        /* linear */
        /* New command received */
        if (dist_command_received) {
            /* Acknowledge the "new_command" message */
            dist_command_received = FALSE;
            /* Reset time */
            linear_t = 0.0;
            /* Warn the control thread that a new command has been received */
            dist_command_updated = TRUE;
        }
        else {
          linear_t += (float)INT_POS_PERIOD / 1000.0;
        }

        /* acceleration and speed are in cm, goal.mean_dist and target_dist in mm
        as all must be in the same unit, we convert to mm
        seee inc/settings.h */
        target_dist = compute_target(linear_t, settings.max_linear_acceleration * 10,
                                        settings.max_linear_acceleration * 10,
                                        settings.cruise_linear_speed * 10,
                                        goal.mean_dist);

        /* angular */
        if (goal.heading != prev_goal_heading) {
            /* New command received */
            angular_t = 0.0;
            prev_goal_heading = goal.heading;
            initial_heading = orientation;

            delta_heading = goal.heading - orientation;
            if (delta_heading < -(HEADING_MAX_VALUE / 2)) {
                delta_heading += HEADING_MAX_VALUE;
            } else if (delta_heading > (HEADING_MAX_VALUE / 2)) {
                delta_heading -= HEADING_MAX_VALUE;
            }
        }
        else {
          angular_t += (float)INT_POS_PERIOD / 1000.0;
        }

        /* Multiply by 16 to convert degree to IMU unit */
        tmp_target_heading = initial_heading + ((int16_t) compute_target(angular_t,
                                          (float) settings.max_angular_acceleration * 16,
                                          (float) settings.max_angular_acceleration * 16,
                                          (float) settings.cruise_angular_speed * 16,
                                          delta_heading));

        if (tmp_target_heading < 0) {
            target_heading = tmp_target_heading + HEADING_MAX_VALUE;
        } else if (tmp_target_heading >= HEADING_MAX_VALUE) {
            target_heading = tmp_target_heading - HEADING_MAX_VALUE;
        } else {
            target_heading = tmp_target_heading;
        }

        // counter just not to spam the console
        static int cpt_print = 0;
        if (cpt_print++ % 20 == 0) {
          LOG_DEBUG("cur_pos = (%.3f, %.3f)\n", (float) cur_pos.x, cur_pos.y);
          LOG_DEBUG("target %d / %d (%d) %d / %d (%d)\r\n", target_heading, goal.heading, orientation, target_dist, goal.mean_dist, current_distance);
        }
        /* Wait to reach the desired period */
        chThdSleepMilliseconds(INT_POS_PERIOD - ST2MS(chVTGetSystemTime() - start_time));
    }
}

extern THD_FUNCTION(control_thread, p) {
    (void)p;

    control_values_t linear_control = {0, 0, 0};
    control_values_t angular_control = {0, 0, 0};

    pid_t linear_pid;
    pid_t angular_pid;

    /* Saved ticks value (to compute current distance) */
    ticks_t saved_ticks = {0, 0};

    /* Previous goal heading, used to know whether a new instruction has been received */
    int16_t prev_goal_heading;

    /* Current heading to maintain (initial orientation before sync_dist, target_heading after) */
    int16_t cur_target_heading;

    /* Commands for motors and other related local variables */
    int32_t prev_command[2] = {0, 0};
    int32_t command[2];
    int32_t prev_linear_command = 0;
    int32_t linear_command = 0;
    int32_t prev_angular_command = 0;
    int32_t angular_command = 0;
    int32_t tmp_command;

    /* Maximum delta on each command between two successive loop turn */
    int32_t max_linear_delta_pwm_command; /* max_linear_acceleration * CONTROL_PERIOD */
    int32_t max_angular_delta_pwm_command; /* max_angular_acceleration * CONTROL_PERIOD */

    /* Start time of the current loop */
    uint32_t start_time;

    int32_t remaining_time;

    uint32_t log_counter = 0;

    /* Initialise the variables */
    prev_goal_heading = goal.heading;
    cur_target_heading = target_heading;

    /* Infinite loop */
    while (TRUE) {

        start_time = chVTGetSystemTime();

        /* Reset the linear PID sum and saved_ticks if a new instruction has been
           received from master */
        if (dist_command_updated) {
            /* Acknowledge the "command_updated" message */
            dist_command_updated = FALSE;

            /* Reset the measured values */
            linear_control.epsilon_sum = 0;
            saved_ticks.left = left_ticks;
            saved_ticks.right = right_ticks;
        }

        /* Reset the angular PID sum if a new instruction has been received from master */
        if (prev_goal_heading != goal.heading) {
            prev_goal_heading = goal.heading;
            angular_control.epsilon_sum = 0;
        }

        /* (Re)Compute the settings value, in case max accelerations have changed */
        max_linear_delta_pwm_command = settings.max_linear_acceleration * CONTROL_PERIOD / 10;
        max_angular_delta_pwm_command = settings.max_angular_acceleration * CONTROL_PERIOD / 10;

        /* Update current_distance, in mm */
        current_distance = 1000 * ((left_ticks - saved_ticks.left) + (right_ticks - saved_ticks.right)) / (2 * settings.ticks_per_m);

        /* Check whether current move is finished */
        translation_ended = ((uint32_t)ABS(current_distance - goal.mean_dist) < settings.linear_allowance);
        rotation_ended = ((ABS(orientation - goal.heading) % (HEADING_MAX_VALUE / 2)) < settings.angular_allowance);

        /* Compute linear_control.epsilon and related input values */
        linear_control.prev_epsilon = linear_control.epsilon;
        linear_control.epsilon = target_dist - current_distance;
        linear_control.epsilon_sum += linear_control.epsilon;

        if (master_stop == FALSE) {
            /* Linear PID */
            linear_pid.p = (settings.linear_coeff.p * linear_control.epsilon) / REDUCTION_FACTOR_P;

            linear_pid.i = (settings.linear_coeff.i * linear_control.epsilon_sum) / REDUCTION_FACTOR_I;

            linear_pid.d = (settings.linear_coeff.d * (linear_control.epsilon - linear_control.prev_epsilon)) / REDUCTION_FACTOR_D;

            prev_linear_command = linear_command;
            linear_command = linear_pid.p + linear_pid.i + linear_pid.d;

            /* Limit linear acceleration/deceleration */
            if ((int32_t)(linear_command - prev_linear_command) > max_linear_delta_pwm_command) {
                linear_command = prev_linear_command + max_linear_delta_pwm_command;
            } else if ((int32_t)(linear_command - prev_linear_command) < -max_linear_delta_pwm_command) {
                linear_command = prev_linear_command - max_linear_delta_pwm_command;
            }

            /* Update current target heading if heading dist sync ref has been reached */
            if (ABS(current_distance) >= goal.heading_dist_sync_ref) {
                cur_target_heading = target_heading;
            }

            /* Compute angular_control.epsilon and related input values */
            angular_control.prev_epsilon = angular_control.epsilon;
            angular_control.epsilon = cur_target_heading - orientation;

            /* Angles are module HEADING_MAX_VALUE, this case must thus be handled
               for the robot to turn in the right direction (the shorter one). */
            if (angular_control.epsilon > (HEADING_MAX_VALUE / 2)) {
                angular_control.epsilon -= HEADING_MAX_VALUE;
            } else if (angular_control.epsilon < -(HEADING_MAX_VALUE / 2)) {
                angular_control.epsilon += HEADING_MAX_VALUE;
            }

            angular_control.epsilon_sum += angular_control.epsilon;

            /* Angular PID */
            angular_pid.p = (settings.angular_coeff.p * angular_control.epsilon) / REDUCTION_FACTOR_P;

            angular_pid.i = (settings.angular_coeff.i * angular_control.epsilon_sum) / REDUCTION_FACTOR_I;

            angular_pid.d = (settings.angular_coeff.d * (angular_control.epsilon - angular_control.prev_epsilon)) / REDUCTION_FACTOR_D;

            prev_angular_command = angular_command;
            angular_command = angular_pid.p + angular_pid.i + angular_pid.d;

            /* Limit angular acceleration/deceleration */
            if ((int32_t)(angular_command - prev_angular_command) > max_angular_delta_pwm_command) {
                angular_command = prev_angular_command + max_angular_delta_pwm_command;
            } else if ((int32_t)(angular_command - prev_angular_command) < -max_angular_delta_pwm_command) {
                angular_command = prev_angular_command - max_angular_delta_pwm_command;
            }

            if (FALSE == orientation_control) {
                LOG_VERBOSE(".");
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

            if (log_counter++ == PID_INFO_PERIOD_FACTOR){
              log_counter = 0;
            //the last term corresponds to acceleration saturation
            //printf("%d\n", max_linear_delta_pwm_command);
            LOG_PID_INFO("[LINEAR]: %d %d %d %d\n", linear_pid.p, linear_pid.i, linear_pid.d,
                (int32_t) (1000 * ((float) (linear_command - prev_linear_command)) / max_linear_delta_pwm_command));
            LOG_PID_INFO("[ANGULAR]: %d %d %d %d\n", angular_pid.p, angular_pid.i, angular_pid.d,
                (int32_t) (1000 * ((float) (angular_command - prev_angular_command)) / max_angular_delta_pwm_command));
            LOG_PID_INFO("[GLOBAL]: %d %d\n",
                (int32_t) (1000 * ((float) command[MOTOR_LEFT]) / MAX_PWM),  //actually in range [0, 2]
                (int32_t) (1000 * ((float) command[MOTOR_RIGHT]) / MAX_PWM));
            LOG_PID_INFO("[MOTORS] %d %d\n", command[MOTOR_LEFT], command[MOTOR_RIGHT]);
            LOG_PID_INFO("[POSITION]: %d %d %d %d\n", target_dist, current_distance, target_heading, orientation);
          }

            /* Apply new commands */
            motor_t motor;
            for (motor = MOTOR_LEFT; motor <= MOTOR_RIGHT; ++motor) {
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
        } else {
            linear_control.epsilon_sum = 0;
            angular_control.epsilon_sum = 0;
#if BASIC_STOP
            motor_set_speed(MOTOR_LEFT, 0U);
            motor_set_speed(MOTOR_RIGHT, 0U);
            linear_command = 0;
            angular_command = 0;
#else
            /* Reduce linear command as much as possible */
            tmp_command = SIGN(linear_command) * (ABS(linear_command) - max_linear_delta_pwm_command);
            if (SIGN(tmp_command) != SIGN(linear_command)) {
                linear_command = 0;
            } else {
                linear_command = tmp_command;
            }

            /* Reduce angular command as much as possible */
            tmp_command = SIGN(angular_command) * (ABS(angular_command) - max_angular_delta_pwm_command);
            if (SIGN(tmp_command) != SIGN(angular_command)) {
                angular_command = 0;
            } else {
                angular_command = tmp_command;
            }

            command[MOTOR_RIGHT] = linear_command + angular_command;
            command[MOTOR_LEFT] = linear_command - angular_command;

            motor_set_speed(MOTOR_LEFT, command[MOTOR_LEFT]);
            motor_set_speed(MOTOR_RIGHT, command[MOTOR_RIGHT]);

            goal.mean_dist -= current_distance;
            dist_command_received = TRUE;
#endif /* BASIC_STOP */
        }

        /* Sleep until next period */
        remaining_time = CONTROL_PERIOD - ST2MS(chVTGetSystemTime() - start_time);
        if (remaining_time > 0) {
            chThdSleepMilliseconds(remaining_time);
        }
    }
}

extern THD_FUNCTION(reset_pos_thread, p) {
    int32_t saved_current_distance;
    uint32_t start_time;

    (void)p;

    while (TRUE) {
        chBSemWait(&reset_orientation_sem);

        /* Disable orientation control */
        orientation_control = FALSE;

        /* Move as fast as possible in the required direction */
        switch(reset_orientation_direction) {
        case 1:   //forward
          goal.mean_dist = 0x0FFFFFFF;
          break;
        case 0:   // backward
          goal.mean_dist = -1 * 0x0FFFFFFF;
          break;
        default:
          LOG_ERROR("WARNING: unknown value of reset_orientation_direction: %d\n",
                  reset_orientation_direction);
        }

        LOG_DEBUG("GOAL = %d; \treset_orient_dir = %d\n", goal.mean_dist,
                                                  reset_orientation_direction);
        dist_command_received = TRUE;
        saved_current_distance = 0;

        start_time = chVTGetSystemTime();
        while (TRUE) {
            chThdSleepMilliseconds(RESET_PERIOD);
            if ((ABS(current_distance) >= 3 && saved_current_distance == current_distance)
                || (ST2MS(chVTGetSystemTime() - start_time) > MAX_DELAY_TO_RETURN)){
                /* no move in the last period */
                set_orientation(reset_orientation_orientation);
                orientation_control = TRUE;
                goal.mean_dist = 0;
                dist_command_received = TRUE;
                break;
            } else {
                saved_current_distance = current_distance;
            }
        }

    }
}

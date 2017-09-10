#include "control.h"
#include "coding_wheels.h"
#include "motor.h"
#include "settings.h"
#include "orientation.h"
#include "position.h"
#include "RTT/SEGGER_RTT.h"
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

/* Period of the control thread, in ms */
#define CONTROL_PERIOD 1

/* Returns the absolute value of the parameter */
#define ABS(x) ((x > 0) ? x : -x)

/* Returns the sign of the parameter (1 or -1) */
#define SIGN(x) ((x < 0) ? -1 : 1)

/* Selects the "urgent stop" strategy */
#define BASIC_STOP 0

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
static volatile int16_t target_heading;

/* Threads stacks */
THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);
THD_WORKING_AREA(wa_int_pos, INT_POS_STACK_SIZE);

/******************************************************************************/
/*                         Public functions                                   */
/******************************************************************************/
/*
prend en argument :
    les caract�ristiques de la courbe de vitesse, � savoir :
        a_montante : acc�l�ration lors de la phase d'augmentation de vitesse
        a_descendante : acc�l�ration lors de la phase de freinage
        v_croisiere : vitesse de croisi�re
        x_final : la destination du robot

    NB : a_montante et a_descendante doivent �tre de signe oppos�
    (a_montante peut �tre n�gative dans le cas o� le robot recule)

renvoie la position � l'instant t

pour memoire, la courbe de vitesse ressemble � :

^ vitesse
|
|         ---------------------
|        /|                   |\
|       / |                   | \
|------/  |                   |  \-------- -> temps
      |   |                   |  |
     t=0  t1                  t2 t3

(ou l'oppose)

car c'est la courbe qui permet de minimiser le temps pour atteindre x_final
tout en ayant une vitesse continue (en accord avec la physique)

*/
extern THD_FUNCTION(int_pos_thread, p) {
    (void)p;

    /*
     * All times are in seconds.
     * All distances are in millimeters.
     */

    /* Tmp */
    float linear_t = 0.0; /* in s */
    float linear_a_montante = 0.0; /* in mm.s-2 */
    float linear_a_descendante = 0.0; /* in mm.s-2 */
    float linear_v_croisiere = 0.0; /* in mm.s-1 */
    float x_final; /* in mm */
    float linear_t1 = 0.0;
    float linear_t2 = 0.0;
    float linear_t3 = 0.0;
    static float linear_t4_carre = 0.0;

    float prev_goal_heading = 0.0;
    float angular_t = 0.0; /* in s */
    float angular_a_montante = 0.0; /* in °.s-2 */
    float angular_a_descendante = 0.0; /* in °.s-2 */
    float angular_v_croisiere = 0.0; /* in °.s-1 */
    float heading_final; /* in IMU unit */
    float delta_heading = 0.0;
    int16_t initial_heading = 0.0;
    float angular_t1 = 0.0;
    float angular_t2 = 0.0;
    float angular_t3 = 0.0;
    float tmp_target_heading;
    static float angular_t4_carre = 0.0;
    float delta;

    uint32_t start_time;

    while (TRUE) {
        start_time = chVTGetSystemTime();

        /* Acquire sensors data and update localisation */
        compute_movement();
        update_orientation();
        update_position();

        /* linear */
        linear_t += (float)INT_POS_PERIOD / 1000.0;

        x_final = (float)goal.mean_dist;
        if (dist_command_received) {
            /* New command received */

            /* Acknowledge the "new_command" message */
            dist_command_received = FALSE;

            /* Reset time */
            linear_t = 0.0;

            /* Update settings */
            linear_a_montante = (float)settings.max_linear_acceleration;
            linear_a_descendante = -(float)settings.max_linear_acceleration;
            linear_v_croisiere = (float)settings.cruise_linear_speed;

            /* Compute values */
            // t1 = instant auquel on atteint la vitesse de croisiere
            // (ie acceleration terminee)
            linear_t1 = linear_v_croisiere / linear_a_montante;

            // t2 = instant auquel on quitte la vitesse de croisiere
            // ie debut du freinage
            linear_t2 = ABS(x_final) / linear_v_croisiere + linear_v_croisiere / 2 * (1 / linear_a_montante + 1 / linear_a_descendante);

            /* calcul du carre de l'instant auquel on passe de la phase
            d'acceleration a la phase de freinage */
            linear_t4_carre = 2 * ABS(x_final) / (linear_a_montante * (1 - linear_a_montante / linear_a_descendante));

            /* Warn the control thread that a new command has been received */
            dist_command_updated = TRUE;
        }

        //cas o� on atteint jamais la vitesse de croisi�re
        if (linear_t2 <= linear_t1) {
            if (linear_t * linear_t <= linear_t4_carre) {
                target_dist = SIGN(x_final) * (int32_t)(linear_a_montante * linear_t * linear_t / 2);
            } else {
                float linear_t4_inv = 2 / sqrt(linear_t4_carre);
                float linear_delta_t = linear_t - linear_t4_inv * ABS(x_final) / linear_a_montante;

                if (linear_delta_t >= 0) {
                    target_dist = (int32_t)x_final;
                } else {
                    target_dist = (int32_t)(SIGN(x_final) * linear_a_descendante / 2 * linear_delta_t * linear_delta_t + x_final);
                }
            }
        } else {
            linear_t3 = linear_t2 - linear_v_croisiere / linear_a_descendante;

            if (linear_t < 0) {        //avant le demarrage
                target_dist = 0;
            } else if (linear_t <= linear_t1 && linear_t <= linear_t2) {    //pendant la phase d'acceleration
                target_dist = SIGN(x_final) * (int32_t)(linear_a_montante * linear_t * linear_t / 2);
            } else if (linear_t <= linear_t2) {               //pendant la phase de croisiere
                target_dist = SIGN(x_final) * (int32_t)(linear_t1 * linear_v_croisiere / 2 + linear_v_croisiere * (linear_t - linear_t1));
            } else if (linear_t <= linear_t3) {               //pendant le freinage
                target_dist = (int32_t)(x_final + SIGN(x_final) * linear_v_croisiere * linear_v_croisiere / 2 / linear_a_descendante \
                    + (linear_t - linear_t2) * (linear_v_croisiere + linear_a_descendante / 2 * (linear_t - linear_t2)));
            } else {                            //apres etre arrive
                target_dist = (int32_t)x_final;
            }
        }



        /* angular */
        angular_t += (float)INT_POS_PERIOD / 1000.0;

        heading_final = (float)goal.heading;
        if (heading_final != prev_goal_heading) {
            /* New command received */
            prev_goal_heading = goal.heading;
            initial_heading = orientation;

            delta_heading = heading_final - (float)orientation;
            if (delta_heading < -(HEADING_MAX_VALUE / 2)) {
                delta_heading += HEADING_MAX_VALUE;
            } else if (delta_heading > (HEADING_MAX_VALUE / 2)) {
                delta_heading -= HEADING_MAX_VALUE;
            }

            /* Reset time */
            angular_t = 0.0;

            /* Update settings */
            /* Multiply by 16 to convert degree to IMU unit */
            angular_a_montante = (float)settings.max_angular_acceleration * 16;
            angular_a_descendante = -(float)settings.max_angular_acceleration * 16;
            angular_v_croisiere = (float)settings.cruise_angular_speed * 16;

            /* Compute values */
            // t1 = instant auquel on atteint la vitesse de croisiere
            // (ie acceleration terminee)
            angular_t1 = angular_v_croisiere / angular_a_montante;

            // t2 = instant auquel on quitte la vitesse de croisiere
            // ie debut du freinage
            angular_t2 = ABS(delta_heading) / angular_v_croisiere + angular_v_croisiere / 2 * (1 / angular_a_montante + 1 / angular_a_descendante);

            /* calcul du carre de l'instant auquel on passe de la phase
            d'acceleration a la phase de freinage */
            angular_t4_carre = 2 * ABS(delta_heading) / (angular_a_montante * (1 - angular_a_montante / angular_a_descendante));
        }

        //cas o� on atteint jamais la vitesse de croisi�re
        if (angular_t2 <= angular_t1) {
            //printf("never cruise\r\n");
            if (angular_t * angular_t <= angular_t4_carre) {
                tmp_target_heading = initial_heading + SIGN(delta_heading) * (int16_t)(angular_a_montante * angular_t * angular_t / 2);
            } else {
                float angular_t4_inv = 2 / sqrt(angular_t4_carre);
                float angular_delta_t = angular_t - angular_t4_inv * ABS(delta_heading) / angular_a_montante;

                if (angular_delta_t >= 0) {
                    tmp_target_heading = (int16_t)heading_final;
                } else {
                    tmp_target_heading = (int16_t)(SIGN(delta_heading) * angular_a_descendante / 2 * angular_delta_t * angular_delta_t + heading_final);
                }
            }
        } else {
            angular_t3 = angular_t2 - angular_v_croisiere / angular_a_descendante;

            if (angular_t < 0) {        //avant le demarrage
                tmp_target_heading = initial_heading;
            } else if (angular_t <= angular_t1 && angular_t <= angular_t2) {    //pendant la phase d'acceleration
                tmp_target_heading = initial_heading + SIGN(delta_heading) * (int16_t)(angular_a_montante * angular_t * angular_t / 2);
            } else if (angular_t <= angular_t2) {               //pendant la phase de croisiere
                tmp_target_heading = initial_heading + SIGN(delta_heading) * (int16_t)(angular_t1 * angular_v_croisiere / 2 + angular_v_croisiere * (angular_t - angular_t1));
            } else if (angular_t <= angular_t3) {               //pendant le freinage
                delta = (heading_final + angular_v_croisiere * angular_v_croisiere / (2 * SIGN(delta_heading) * angular_a_descendante) + (angular_t - angular_t2) * (SIGN(delta_heading) * angular_v_croisiere + SIGN(delta_heading) * angular_a_descendante * (angular_t - angular_t2) / 2));
                tmp_target_heading = initial_heading + delta;
            } else {                            //apres etre arrive
                tmp_target_heading = (int16_t)heading_final;
            }
        }

        if (tmp_target_heading < 0.0) {
            target_heading = tmp_target_heading + HEADING_MAX_VALUE;
        } else if (tmp_target_heading > HEADING_MAX_VALUE) {
            target_heading = tmp_target_heading - HEADING_MAX_VALUE;
        } else {
            target_heading = tmp_target_heading;
        }

        printf("target %d / %d (%d) %d / %d (%d)\r\n", target_heading, goal.heading, orientation, target_dist, goal.mean_dist, current_distance);

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

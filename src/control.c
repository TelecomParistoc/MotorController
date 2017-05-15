#include "control.h"
#include "coding_wheels.h"
#include "motor.h"
#include "settings.h"
#include "orientation.h"
#include "position.h"
#include "RTT/SEGGER_RTT.h"

#define MAX_PWM 70

#define ABS(x) ((x > 0) ? x : -x)
#define SIGN(x) ((x < 0) ? -1 : 1)

/* Target values */
volatile int16_t goal_mean_dist;
volatile uint16_t goal_heading;
volatile int16_t heading_dist_sync_ref;

/* Current travelled since last goal_mean_dist update */
volatile int32_t current_distance;

/*
 * PID coeffs.
 * Values will be divided by the corresponding REDUCTION_FACTOR.
 */
/* Linear PID coeffs */
volatile uint16_t linear_p_coeff;
volatile uint16_t linear_i_coeff;
volatile uint16_t linear_d_coeff;

/* Angular PID coeffs */
volatile uint16_t angular_p_coeff;
volatile uint16_t angular_i_coeff;
volatile uint16_t angular_d_coeff;

volatile uint8_t master_stop;

/* Intermediate value computed by the int_pos thread */
static volatile int32_t tmp_target_dist;

static volatile int16_t tmp_target_heading;

#define REDUCTION_FACTOR_P 1000
#define REDUCTION_FACTOR_I 10000
#define REDUCTION_FACTOR_D 10000

/* Threads stacks */
THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);
THD_WORKING_AREA(wa_int_pos, INT_POS_STACK_SIZE);

int32_t max_linear_delta_pwm_command; /* max_linear_acceleration * CONTROL_PERIOD / alpha with v = alpha * PWM */
int32_t max_angular_delta_pwm_command;

#define INT_POS_PERIOD 10 /* in ms */
#define CONTROL_PERIOD 1 /* in ms */
#define ALPHA 10

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

    /* Tmp */
    float prev_goal_dist = 0.0;
    float linear_t = 0.0; /* in s */
    float linear_a_montante = 0.0;
    float linear_a_descendante = 0.0;
    float linear_v_croisiere = 0.0;
    float x_final;
    float linear_t1 = 0.0;
    float linear_t2 = 0.0;
    float linear_t3 = 0.0;
    static float linear_t4_carre = 0.0;
    static int linear_i;

    float prev_goal_heading;
    float angular_t = 0.0; /* in s */
    float angular_a_montante = 0.0;
    float angular_a_descendante = 0.0;
    float angular_v_croisiere = 0.0;
    float heading_final;
    float delta_heading = 0.0;
    int16_t initial_heading = 0.0;
    float angular_t1 = 0.0;
    float angular_t2 = 0.0;
    float angular_t3 = 0.0;
    static float angular_t4_carre = 0.0;
    static int angular_i;

    while (TRUE) {
        chThdSleepMilliseconds(INT_POS_PERIOD);
        //printf("t= %d target: %d (%d)\r\n", (int)t, tmp_target_dist, goal_mean_dist);

        /* linear */
        linear_t += (float)INT_POS_PERIOD / 1000.0;

        x_final = (float)goal_mean_dist;
        if (x_final != prev_goal_dist) {
            /* New command received */
            prev_goal_dist = goal_mean_dist;

            /* Reset time */
            linear_t = 0.0;

            /* Update settings */
            linear_a_montante = (float)max_linear_acceleration;
            linear_a_descendante = -(float)max_linear_acceleration;
            linear_v_croisiere = (float)cruise_linear_speed;

            /* Compute values */
            // t1 = instant auquel on atteint la vitesse de croisiere
            // (ie acceleration terminee)
            linear_t1 = linear_v_croisiere / linear_a_montante;

            // t2 = instant auquel on quitte la vitesse de croisiere
            // ie debut du freinage
            linear_t2 = x_final / linear_v_croisiere + linear_v_croisiere / 2 * (1 / linear_a_montante + 1 / linear_a_descendante);

            /* calcul du carre de l'instant auquel on passe de la phase
            d'acceleration a la phase de freinage */
            linear_t4_carre = 2 * x_final / (linear_a_montante * (1 - linear_a_montante / linear_a_descendante));
        }

        //cas o� on atteint jamais la vitesse de croisi�re
        if (linear_t2 <= linear_t1) {
            if (linear_t * linear_t <= linear_t4_carre) {
                tmp_target_dist = SIGN(x_final) * (int32_t)(linear_a_montante * linear_t * linear_t / 2);
            } else {
                //calcul (demoniaque) de 2 / sqrt(t4_carre)
                //noter qu'il s'agit quand m�me d'une valeur approch�e...
                //mais avec 3 d�cimales exactes
                linear_i = *(int*)&linear_t4_carre;
                linear_i = 0x5f3759df - (linear_i >> 1);
                float linear_t4_inv = *(float*)&linear_i;
                linear_t4_inv = linear_t4_inv * (3.0f - (linear_t4_carre * linear_t4_inv * linear_t4_inv));

                float linear_delta_t = linear_t - linear_t4_inv * x_final / linear_a_montante;

                if (linear_delta_t >= 0) {
                    tmp_target_dist = (int32_t)x_final;
                } else {
                    tmp_target_dist = (int32_t)(SIGN(x_final) * linear_a_descendante / 2 * linear_delta_t * linear_delta_t + x_final);
                }
            }
        } else {
            linear_t3 = linear_t2 - linear_v_croisiere / linear_a_descendante;

            if (linear_t < 0) {        //avant le demarrage
                tmp_target_dist = 0;
            } else if (linear_t <= linear_t1 && linear_t <= linear_t2) {    //pendant la phase d'acceleration
                tmp_target_dist = SIGN(x_final) * (int32_t)(linear_a_montante * linear_t * linear_t / 2);
            } else if (linear_t <= linear_t2) {               //pendant la phase de croisiere
                tmp_target_dist = SIGN(x_final) * (int32_t)(linear_t1 * linear_v_croisiere / 2 + linear_v_croisiere * (linear_t - linear_t1));
            } else if (linear_t <= linear_t3) {               //pendant le freinage
                tmp_target_dist = (int32_t)(x_final + SIGN(x_final) * linear_v_croisiere * linear_v_croisiere / 2 / linear_a_descendante \
                    + (linear_t - linear_t2) * (linear_v_croisiere + linear_a_descendante / 2 * (linear_t - linear_t2)));
            } else {                            //apres etre arrive
                tmp_target_dist = (int32_t)x_final;
            }
        }



        /* angular */
        angular_t += (float)INT_POS_PERIOD / 1000.0;

        heading_final = (float)goal_heading;
        if (heading_final != prev_goal_heading) {
            /* New command received */
            prev_goal_heading = goal_heading;
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
            angular_a_montante = (float)max_angular_acceleration;
            angular_a_descendante = -(float)max_angular_acceleration;
            angular_v_croisiere = (float)cruise_angular_speed;

            /* Compute values */
            // t1 = instant auquel on atteint la vitesse de croisiere
            // (ie acceleration terminee)
            angular_t1 = angular_v_croisiere / angular_a_montante;

            // t2 = instant auquel on quitte la vitesse de croisiere
            // ie debut du freinage
            angular_t2 = delta_heading / angular_v_croisiere + angular_v_croisiere / 2 * (1 / angular_a_montante + 1 / angular_a_descendante);

            /* calcul du carre de l'instant auquel on passe de la phase
            d'acceleration a la phase de freinage */
            angular_t4_carre = 2 * delta_heading / (angular_a_montante * (1 - angular_a_montante / angular_a_descendante));
        }

        //cas o� on atteint jamais la vitesse de croisi�re
        if (angular_t2 <= angular_t1) {
            if (angular_t * angular_t <= angular_t4_carre) {
                tmp_target_heading = initial_heading + SIGN(delta_heading) * (int16_t)(angular_a_montante * angular_t * angular_t / 2);
            } else {
                //calcul (demoniaque) de 2 / sqrt(t4_carre)
                //noter qu'il s'agit quand m�me d'une valeur approch�e...
                //mais avec 3 d�cimales exactes
                angular_i = *(int*)&angular_t4_carre;
                angular_i = 0x5f3759df - (angular_i >> 1);
                float angular_t4_inv = *(float*)&angular_i;
                angular_t4_inv = angular_t4_inv * (3.0f - (angular_t4_carre * angular_t4_inv * angular_t4_inv));

                float angular_delta_t = angular_t - angular_t4_inv * heading_final / angular_a_montante;

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
                tmp_target_heading = (int16_t)(heading_final + SIGN(delta_heading) * angular_v_croisiere * angular_v_croisiere / 2 / angular_a_descendante \
                    + (angular_t - angular_t2) * (angular_v_croisiere + angular_a_descendante / 2 * (angular_t - angular_t2)));
            } else {                            //apres etre arrive
                tmp_target_dist = (int16_t)heading_final;
            }
        }

        if (tmp_target_heading < 0.0) {
            tmp_target_heading += HEADING_MAX_VALUE;
        }
    }
}

extern THD_FUNCTION(control_thread, p) {
    (void)p;

    /* Linear values */
    int32_t prev_linear_epsilon = 0;
    int32_t linear_epsilon = 0;
    int32_t linear_epsilon_sum = 0;

    /* Angular values */
    int32_t prev_angular_epsilon = 0;
    int32_t angular_epsilon = 0;
    int32_t angular_epsilon_sum = 0;

    /* Linear PID  */
    int32_t linear_p;
    int32_t linear_i;
    int32_t linear_d;

    /* Angular PID */
    int32_t angular_p;
    int32_t angular_i;
    int32_t angular_d;

    int32_t saved_left_ticks = 0;
    int32_t saved_right_ticks = 0;

    /* Previous target distance, used to know whether a new target has been received */
    int16_t prev_goal_dist;

    /* Current heading to maintain */
    int16_t tmp_goal_heading;

    /* Previous target heading, used to know whether a new target has been received */
    int16_t prev_goal_heading;

    /* Commands for motors and other related local variables */
    int32_t prev_command[2] = {0, 0};
    int32_t command[2];
    int32_t prev_linear_command = 0;
    int32_t linear_command = 0;
    int32_t prev_angular_command = 0;
    int32_t angular_command = 0;
    int32_t tmp_command;

    /* Initialise the variables */
    prev_goal_dist = goal_mean_dist;
    prev_goal_heading = goal_heading;
    tmp_goal_heading = goal_heading;

    while (TRUE) {

        compute_movement();
        update_orientation();

        /* Reset the linear PID sum and saved_ticks if a new target has been
           received from master */
        if (prev_goal_dist != goal_mean_dist) {
            prev_goal_dist = goal_mean_dist;
            linear_epsilon_sum = 0;
            saved_left_ticks = left_ticks;
            saved_right_ticks = right_ticks;
        }

        /* Reset the angular PID sum if a new target has been received from master */
        if (prev_goal_heading != goal_heading) {
            prev_goal_heading = goal_heading;
            angular_epsilon_sum = 0;
        }

        /* Compute the settings value, in case max accelerations have changed */
        max_linear_delta_pwm_command = max_linear_acceleration * CONTROL_PERIOD / ALPHA;
        max_angular_delta_pwm_command = max_angular_acceleration * CONTROL_PERIOD / ALPHA;

        /* Compute linear_epsilon and related input values */
        prev_linear_epsilon = linear_epsilon;
        current_distance = 1000 * ((left_ticks - saved_left_ticks) + (right_ticks - saved_right_ticks)) / (2 * ticks_per_m); /* In mm */
        linear_epsilon = tmp_target_dist - current_distance;
        linear_epsilon_sum += linear_epsilon;
        printf("current %d\r\n", current_distance);

        /* Linear PID */
        linear_p = (linear_p_coeff * linear_epsilon) / REDUCTION_FACTOR_P;

        linear_i = (linear_i_coeff * linear_epsilon_sum) / REDUCTION_FACTOR_I;

        linear_d = (linear_d_coeff * (linear_epsilon - prev_linear_epsilon)) / REDUCTION_FACTOR_D;

        prev_linear_command = linear_command;
        linear_command = linear_p + linear_i + linear_d;
        //printf("linear %d (%d, %d, %d)\r\n", linear_command, linear_p, linear_i, linear_d);

        /* Limit linear acceleration/deceleration */
        if ((int32_t)(linear_command - prev_linear_command) > max_linear_delta_pwm_command) {
            linear_command = prev_linear_command + max_linear_delta_pwm_command;
        } else if ((int32_t)(linear_command - prev_linear_command) < -max_linear_delta_pwm_command) {
            linear_command = prev_linear_command - max_linear_delta_pwm_command;
        }
        //printf("limited linear %d\r\n", linear_command);

        /* Update current target heading if heading dist sync ref has been reached */
        if ((heading_dist_sync_ref == 0) || (ABS(current_distance) >= heading_dist_sync_ref)) {
            tmp_goal_heading = goal_heading;
        }

        /* Compute angular_epsilon and related input values */
        prev_angular_epsilon = angular_epsilon;
        angular_epsilon = tmp_goal_heading - orientation;

        /* Angles are module HEADING_MAX_VALUE, this case must thus be handled
           for the robot to turn in the right direction (the shorter one). */
        if (angular_epsilon > (HEADING_MAX_VALUE / 2)) {
            angular_epsilon = HEADING_MAX_VALUE - angular_epsilon;
        } else if (angular_epsilon < -(HEADING_MAX_VALUE / 2)) {
            angular_epsilon += HEADING_MAX_VALUE;
        }

        angular_epsilon_sum += angular_epsilon;
        //printf("ang %d (%d - %d)\r\n", angular_epsilon, goal_heading, orientation);

        /* Angular PID */
        angular_p = (angular_p_coeff * angular_epsilon) / REDUCTION_FACTOR_P;

        angular_i = (angular_i_coeff * angular_epsilon_sum) / REDUCTION_FACTOR_I;

        angular_d = (angular_d_coeff * (angular_epsilon - prev_angular_epsilon)) / REDUCTION_FACTOR_D;

        prev_angular_command = angular_command;
        angular_command = angular_p + angular_i + angular_d;
        //printf("angular %d \r\n", angular_command);

        /* Limit angular acceleration/deceleration */
        if ((int32_t)(angular_command - prev_angular_command) > max_angular_delta_pwm_command) {
            angular_command = prev_angular_command + max_angular_delta_pwm_command;
        } else if ((int32_t)(angular_command - prev_angular_command) < -max_angular_delta_pwm_command) {
            angular_command = prev_angular_command - max_angular_delta_pwm_command;
        }

        /* Motor commands */
        /* If left wheel required speed is too high, reduce both components */
        tmp_command = ABS(linear_command + angular_command);
        //printf("tmp_command %d\r\n", tmp_command);
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

        printf("command: %d || %d\r\n", command[MOTOR_LEFT], command[MOTOR_RIGHT]);
        /* Apply new commands */
        if (master_stop == FALSE) {
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
            motor_set_speed(MOTOR_LEFT, 0U);
            motor_set_speed(MOTOR_RIGHT, 0U);
        }

        /* Sleep until next period */
        chThdSleepMilliseconds(CONTROL_PERIOD);
    }
}

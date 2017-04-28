#include "control.h"

volatile uint16_t linear_p_coeff;
volatile uint16_t linear_i_coeff;
volatile uint16_t linear_d_coeff;
volatile uint16_t angular_p_coeff;
volatile uint16_t angular_i_coeff;
volatile uint16_t angular_d_coeff;

THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);

extern THD_FUNCTION(control_thread, p) {
    (void)p;

    while (TRUE) {
        /* Sampling */

        /* Linear PID */

        /* Angular PID */

        /* Motor command */


        chThdSleepMilliseconds(CONTROL_PERIOD);
    }
}

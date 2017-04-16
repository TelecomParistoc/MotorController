#include "control.h"

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

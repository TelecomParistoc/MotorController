#ifndef CONTROL_H
#define CONTROL_H

#define CONTROL_STACK_SIZE 1024

/*
 * Delay between 2 samplings and corrections, in ms.
 */
#define CONTROL_PERIOD 10

extern THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);

extern THD_FUNCTION(control_thread, p);

#endif /* CONTROL_H */

#ifndef CONTROL_H
#define CONTROL_H

#define CONTROL_STACK_SIZE 1024

/*
 * Delay between 2 samplings and corrections, in ms.
 */
#define CONTROL_PERIOD 10

extern volatile uint16_t linear_p_coeff;
extern volatile uint16_t linear_i_coeff;
extern volatile uint16_t linear_d_coeff;
extern volatile uint16_t angular_p_coeff;
extern volatile uint16_t angular_i_coeff;
extern volatile uint16_t angular_d_coeff;
extern THD_WORKING_AREA(wa_control, CONTROL_STACK_SIZE);

extern THD_FUNCTION(control_thread, p);

#endif /* CONTROL_H */

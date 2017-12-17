#include "motor.h"

#define RCC_TIM17EN 0x00040000
#define RCC_TIM17RST 0x00040000

/*
 * Rotation sense for forward and backward movements, for each motor.
 * MOTOR_LEFT
 *      |-----FORWARD -> ??
 *      |-----BACKWARD -> ??
 * MOTOR_RIGHT
 *      |-----FORWARD -> ??
 *      |-----BACKWARD -> ??
 */
static motor_sense_t rotation_direction[2][2];

/*
 * Current direction for each motor (FORWARD or BACKWARD).
 */
static motor_direction_t motor_direction[2];

/*
 * Pin allocation for each motor.
 */
static const uint8_t pin_A[2] = {GPIOA_RMOTA, GPIOA_LMOTA};
static const uint8_t pin_B[2] = {GPIOA_RMOTB, GPIOA_LMOTB};

static int8_t left_speed = 0U;
static int8_t right_speed = 0U;

#define PWM_FREQUENCY_KHZ 20
#define CLK_KHZ 72000

static PWMConfig pwm_config_tim2 = {
    PWM_FREQUENCY_KHZ * MAX_COMMAND * 1000U,
    MAX_COMMAND,
    NULL,
    {
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_DISABLED, NULL}
    },
    0,
    0
};

extern void motor_init(motor_sense_t motor_left_forward_sense, motor_sense_t motor_right_forward_sense) {

    /* Start PWM TIM 2 */
    pwmStart(&PWMD2, &pwm_config_tim2);

    /* Start PWM TIM17 */
    rccEnableAPB2(RCC_TIM17EN, FALSE);
    TIM17->CCER  = 0x00;       // disable capture/compare during setup
    TIM17->CR2   = 0x00;
	TIM17->BDTR  = 0x8C00;     // enable outputs
	TIM17->DIER  = 0x00;       // disable DMA and interrupts
	TIM17->CCMR1 = 0x68;       // PWM mode 1 on channel 1
	TIM17->ARR   = MAX_COMMAND;
	TIM17->PSC   = CLK_KHZ/(PWM_FREQUENCY_KHZ * MAX_COMMAND) - 1; // setup prescaler
	TIM17->CCR1  = 0;          // turn off motors at startup
	TIM17->CCER  = 0x01;       // enable CC1
	TIM17->EGR   = 0x01;       // generate an update event to load the setup values
	TIM17->CNT   = 0;
    TIM17->CR1 = 0x81; // enable counter

    rotation_direction[MOTOR_LEFT][FORWARD] = motor_left_forward_sense;
    rotation_direction[MOTOR_LEFT][BACKWARD] = DIRECTION_2 - motor_left_forward_sense;
    rotation_direction[MOTOR_RIGHT][FORWARD] = motor_right_forward_sense;
    rotation_direction[MOTOR_RIGHT][BACKWARD] = DIRECTION_2 - motor_right_forward_sense;

    motor_set_direction(MOTOR_LEFT, FORWARD);
    motor_set_direction(MOTOR_RIGHT, FORWARD);
}

extern int motor_set_speed(motor_t motor, uint8_t speed) {
    int status;

    if ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) {
        status = -1;
    } else if (speed > MAX_COMMAND) {
        status = -2;
    } else {
        switch(motor)
        {
        case MOTOR_LEFT:
            pwmEnableChannel(&PWMD2, 2, speed);
            left_speed = speed;
            break;
        case MOTOR_RIGHT:
            TIM17->CCR1 = speed;
            right_speed = speed;
            break;
        default:
            break;
        }
        status = 0;
    }
    return status;
}

extern int8_t motor_get_speed(motor_t motor) {
    switch (motor) {
        case MOTOR_LEFT:
            return left_speed;
        case MOTOR_RIGHT:
            return right_speed;
        default:
            return -1;
    }
}

extern motor_direction_t motor_get_direction(motor_t motor)
{
    if ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) {
        return (motor_direction_t)-1;
    } else{
        return motor_direction[motor];
    }
}

extern int motor_set_direction(motor_t motor, motor_direction_t direction)
{
    int status;
    if ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) {
        status = -1;
    } else if ((direction != FORWARD) && (direction != BACKWARD)) {
        status = -2;
    } else {
        motor_direction[motor] = direction;

        if (rotation_direction[motor][direction] == DIRECTION_1) {
            palSetPad(GPIOA, pin_A[motor]);
            palClearPad(GPIOA, pin_B[motor]);
        } else {
            palClearPad(GPIOA, pin_A[motor]);
            palSetPad(GPIOA, pin_B[motor]);
        }
        status = 0;
    }
    return status;
}

extern void motor_toggle_direction(motor_t motor)
{
    if (motor_direction[motor] == FORWARD) {
        motor_set_direction(motor, BACKWARD);
    } else {
        motor_set_direction(motor, FORWARD);
    }
}

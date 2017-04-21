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

static const uint8_t pin_A[2] = {GPIOA_LMOTA, GPIOA_RMOTA};
static const uint8_t pin_B[2] = {GPIOA_LMOTB, GPIOA_RMOTB};

static PWMConfig pwm_config_tim2 = {
    2000000,
    100,
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

#define PWM_MAX 100
#define PWM_FREQUENCY_KHZ 19
#define CLK_KHZ 72000

extern void init_motor(motor_sense_t motor_left_forward_sense, motor_sense_t motor_right_forward_sense) {

    /* Start PWM TIM 2 */
    pwmStart(&PWMD2, &pwm_config_tim2);

    rccEnableAPB2(RCC_TIM17EN, FALSE);
    TIM17->CCER  = 0x00;       // disable capture/compare during setup
    TIM17->CR2   = 0x00;
	TIM17->BDTR  = 0x8C00;     // enable outputs
	TIM17->DIER  = 0x00;       // disable DMA and interrupts
	TIM17->CCMR1 = 0x68;       // PWM mode 1 on channel 1
	TIM17->ARR   = PWM_MAX;
	TIM17->PSC   = CLK_KHZ/(PWM_FREQUENCY_KHZ * PWM_MAX) - 1; // setup prescaler
	TIM17->CCR1  = 0;          // turn off motors at startup
	TIM17->CCER  = 0x01;       // enable CC1
	TIM17->EGR   = 0x01;       // generate an update event to load the setup values
	TIM17->CNT   = 0;
    TIM17->CR1 = 0x81; // enable counter

    set_direction(MOTOR_LEFT, FORWARD);
    set_direction(MOTOR_RIGHT, FORWARD);

    /* Half speed (for test) */
    set_speed(MOTOR_LEFT, 50);
    set_speed(MOTOR_RIGHT, 50);

    rotation_direction[MOTOR_LEFT][FORWARD] = motor_left_forward_sense;
    rotation_direction[MOTOR_LEFT][BACKWARD] = DIRECTION_2 - motor_left_forward_sense;
    rotation_direction[MOTOR_RIGHT][FORWARD] = motor_right_forward_sense;
    rotation_direction[MOTOR_RIGHT][BACKWARD] = DIRECTION_2 - motor_right_forward_sense;
}

extern int set_speed(motor_t motor, uint32_t speed) {
    if (speed > PWM_MAX)
    {
        return -1;
    }
    switch(motor)
    {
        case MOTOR_LEFT:
            pwmEnableChannel(&PWMD2, 2, speed);
            break;
        case MOTOR_RIGHT:
            TIM17->CCR1 = speed;
            break;
        default:
            break;
    }
    return 0;
}

extern int set_direction(motor_t motor, motor_direction_t direction)
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

extern motor_direction_t get_direction(motor_t motor)
{
    if ((motor != MOTOR_LEFT) && (motor != MOTOR_RIGHT)) {
        return (motor_direction_t)-1;
    } else{
        return motor_direction[motor];
    }
}

extern void toggle_direction(motor_t motor)
{
    if (motor_direction[motor] == FORWARD) {
        set_direction(motor, BACKWARD);
    } else {
        set_direction(motor, FORWARD);
    }
}

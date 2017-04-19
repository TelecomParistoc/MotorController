#include "motor.h"

#define RCC_TIM17EN 0x00040000
#define RCC_TIM17RST 0x00040000

static motor_direction_t motor_direction[2];
static const uint8_t pin_A[2];

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

static PWMDriver PWMD17;

extern void init_motor(void) {

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
	TIM17->CCR1  = 50;          // turn off motors at startup
	TIM17->CCER  = 0x01;       // enable CC1
	TIM17->EGR   = 0x01;       // generate an update event to load the setup values
	TIM17->CNT   = 0;
    TIM17->CR1 = 0x81; // enable counter

    set_direction(MOTOR_LEFT, FORWARD);
    set_direction(MOTOR_RIGHT, FORWARD);

    /* Half speed (for test) */
    set_speed(MOTOR_LEFT, 50);
    //    set_speed(MOTOR_RIGHT, 50);
}

extern void set_speed(motor_t motor, uint32_t speed) {
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
}

extern int set_direction(motor_t motor, motor_direction_t direction)
{
    if ((motor < 0) || (motor > 1))
    {
        return -1;
    }

    motor_direction[MOTOR_LEFT] = direction;

    switch (motor)
    {
        case MOTOR_LEFT:
            if (direction == FORWARD) {
                palSetPad(GPIOA, GPIOA_LMOTA);
                palClearPad(GPIOA, GPIOA_LMOTB);
            } else {
                palClearPad(GPIOA, GPIOA_LMOTA);
                palSetPad(GPIOA, GPIOA_LMOTB);
            }
            break;
        case MOTOR_RIGHT:
            if (direction == FORWARD) {
                palSetPad(GPIOA, GPIOA_RMOTA);
                palClearPad(GPIOA, GPIOA_RMOTB);
            } else {
                palClearPad(GPIOA, GPIOA_RMOTA);
                palSetPad(GPIOA, GPIOA_RMOTB);
            }
            break;
        default:
            break;
    }
    return 0;
}

extern void toggle_direction(motor_t motor)
{
    switch (motor)
    {
        case MOTOR_LEFT:
            if (motor_direction[MOTOR_LEFT] == FORWARD) {
                set_direction(MOTOR_LEFT, BACKWARD);
            } else {
                set_direction(MOTOR_LEFT, FORWARD);
            }
            break;
        case MOTOR_RIGHT:
            if (motor_direction[MOTOR_RIGHT] == FORWARD) {
                set_direction(MOTOR_RIGHT, BACKWARD);
            } else {
                set_direction(MOTOR_RIGHT, FORWARD);
            }
            break;
        default:
            break;
    }
}

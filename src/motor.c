#include "motor.h"
#include "hal.h"

#define RCC_TIM17EN 0x00040000
#define RCC_TIM17RST 0x00040000

static PWMConfig pwm_config_tim2 = {
    1000000,
    1024,
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

static PWMConfig pwm_config_tim17 = {
    1000000,
    1024,
    NULL,
    {
        {PWM_OUTPUT_DISABLED, NULL}
    },
    0,
    0
};

static PWMDriver PWMD17;

extern void init_motor(void) {

    /* Start PWM TIM 2 */
    pwmStart(&PWMD2, &pwm_config_tim2);

    /* Start PWM TIM 17 */
    pwmObjectInit(&PWMD17);
    PWMD17.channels = STM32_TIM17_CHANNELS;
    PWMD17.tim = STM32_TIM17;

    osalSysLock();
    PWMD17.config = &pwm_config_tim17;
    PWMD17.period = pwm_config_tim17.period;

    if (PWMD17.state == PWM_STOP) {
        rccEnableAPB2(RCC_TIM17EN, FALSE);
        rccResetAPB2(RCC_TIM17RST);
        PWMD17.clock = STM32_TIMCLK1;
        pwm_lld_start(&PWMD17);
    }

    PWMD17.enabled = 0;
    PWMD17.state = PWM_READY;
    osalSysLock();

    /* Half speed (for test) */
    pwmEnableChannel(&PWMD2, 2, 512);
    pwmEnableChannel(&PWMD17, 0, 512);
}

extern void set_speed(motor_t motor, uint32_t speed) {
    switch(motor)
    {
        case MOTOR_LEFT:
            pwmEnableChannel(&PWMD2, 2, speed);
            break;
        case MOTOR_RIGHT:
            pwmEnableChannel(&PWMD17, 0, speed);
            break;
        default:
            break;
    }
}

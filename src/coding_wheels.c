#include "coding_wheels.h"

volatile int32_t left_ticks;
volatile int32_t right_ticks;

static wheel_orientation_t right_wheel_orientation;
static wheel_orientation_t left_wheel_orientation;

static void ext_cb(EXTDriver* driver, expchannel_t channel);

const EXTConfig ext_config = {
    {
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, ext_cb}, // Coding wheel A
        {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOB, ext_cb}, // Coding wheel B
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL},
        {EXT_CH_MODE_DISABLED, NULL}
    }
};

/*
 * Update the tick counters.
 */
static void ext_cb(EXTDriver* driver, expchannel_t channel)
{
    (void)driver;
    if (channel == GPIOB_RCODA) {
        if (palReadPad(GPIOB, GPIOB_RCODB) == PAL_LOW) {
            right_wheel_orientation ? right_ticks++ : right_ticks--;
        } else {
            right_wheel_orientation ? right_ticks-- : right_ticks++;
        }
    } else if (channel == GPIOB_LCODA) {
        if (palReadPad(GPIOB, GPIOB_LCODB) == PAL_LOW) {
            left_wheel_orientation ? left_ticks-- : left_ticks++;
        } else {
            left_wheel_orientation ? left_ticks++ : left_ticks--;
        }
    }
}

extern void init_coding_wheels(coding_wheels_config_t config)
{
    /* Initialize the tick counters with the given values */
    left_ticks = config.initial_left_ticks;
    right_ticks = config.initial_right_ticks;

    left_wheel_orientation = config.left_wheel_orientation;
    right_wheel_orientation = config.right_wheel_orientation;

    /* Start the ChibiOS ext driver */
    extStart(&EXTD1, &ext_config);
}

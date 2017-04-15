#include "test_position.h"
#include "position.h"
#include "orientation.h"
#include "settings.h"
#include "test.h"
#include "coding_wheels.h"
#include "../src/RTT/SEGGER_RTT.h"

extern int32_t test_position_0010(void)
{
    int32_t status = TEST_NO_ERROR;

    previous_right_ticks = 0;
    previous_left_ticks = 0;

    left_ticks = 0;
    right_ticks = 0;

    compute_movement();

    if (delta_left != 0) {
        status |= TEST_INVALID_0;
    }

    if (delta_right != 0) {
        status |= TEST_INVALID_1;
    }

    left_ticks = 15;
    right_ticks = 89;

    compute_movement();

    if (delta_left != 15) {
        status |= TEST_INVALID_2;
    }

    if (delta_right != 89)
    {
        status |= TEST_INVALID_3;
    }

    if (previous_right_ticks != right_ticks) {
        status |= TEST_INVALID_4;
    }

    if (previous_left_ticks != left_ticks) {
        status |= TEST_INVALID_5;
    }

    left_ticks = 137;
    right_ticks = 43;

    compute_movement();

    if (delta_left != 122) {
        status |= TEST_INVALID_6;
    }

    if (delta_right != -46) {
        status |= TEST_INVALID_7;
    }

    left_ticks = 2789;
    right_ticks = 98622;

    if (previous_left_ticks != 137) {
        status |= TEST_INVALID_8;
    }

    if (previous_right_ticks != 43) {
        status |= TEST_INVALID_9;
    }

    return status;
}

extern void test_position_0020(void)
{
    angular_trust_threshold = 0;
    wheels_gap = 5;
    ticks_per_cm = 50;
    orientation = 0;

    previous_right_ticks = 0;
    previous_left_ticks = 0;

    current_x = 0;
    current_y = 0;

    // Tout droit
    right_ticks = 500;
    left_ticks = 500;

    compute_movement();
    update_orientation();
    update_position();

    printf("[POSITION 0020] 1: x %d (100) y %d (0)\r\n", current_x, current_y);

    left_ticks += 196;
    right_ticks += 589;

    compute_movement();
    update_orientation();
    printf ("orientation %d\r\n", orientation);
    printf ("delta_alpha %d\r\n", delta_alpha);
    update_position();

    printf("[POSITION 0020] 2: x %d (150) y %d (50)\r\n", current_x, current_y);

    left_ticks += 196;
    right_ticks += 589;

    compute_movement();
    update_orientation();
    printf ("orientation %d\r\n", orientation);
    printf ("delta_alpha %d\r\n", delta_alpha);
    update_position();

    printf("[POSITION 0020] 3: x %d (100) y %d (100)\r\n", current_x, current_y);

    left_ticks += 98;
    right_ticks += 295;

    compute_movement();
    update_orientation();
    printf ("orientation %d\r\n", orientation);
    printf ("delta_alpha %d\r\n", delta_alpha);
    update_position();

    printf("[POSITION 0020] 4: x %d (65) y %d (85)\r\n", current_x, current_y);

}

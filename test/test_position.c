#include "test_position.h"
#include "position.h"
#include "test.h"

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

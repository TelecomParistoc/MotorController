#include "test_orientation.h"
#include "orientation.h"
#include "position.h"
#include "settings.h"
#include "test.h"
#include "coding_wheels.h"
#include "../src/RTT/SEGGER_RTT.h"

extern int32_t test_orientation(void)
{
    int32_t status;
    int32_t ret_value;

    ret_value = test_orientation_0010();
    if (ret_value != TEST_NO_ERROR) {
        status |= TEST_INVALID_0;
    }

    return status;
}

extern int32_t test_orientation_0010(void)
{
    int32_t stat = TEST_NO_ERROR;

    angular_trust_threshold = 0;
    wheels_gap = 5;
    ticks_per_cm = 50;
    orientation = 0;

    previous_left_ticks = 0;
    previous_right_ticks = 0;

    left_ticks = 0;
    right_ticks = 392;

    compute_movement();
    update_orientation();

    if ((delta_alpha < 1410) || (delta_alpha > 1416)) {
        stat |= TEST_INVALID_0;
    } else {
        printf("no error\r\n");
    }

    printf("[ORIENTATION 0010] 1: delta_alpha: %d (expected 1411) \r\n", delta_alpha);

    left_ticks = 392;

    compute_movement();
    update_orientation();

    if ((delta_alpha < -1416) || (delta_alpha > -1410)) {
        stat |= TEST_INVALID_1;
    }

    printf("[ORIENTATION 0010] 2: delta_alpha %d (expected -1411) \r\n", delta_alpha);

    left_ticks = 500;
    right_ticks = 500;

    compute_movement();
    update_orientation();

    printf("[ORIENTATION 0010] 3: delta_alpha %d (expected 0)\r\n", delta_alpha);

    left_ticks += 1570;
    right_ticks += 2355;

    compute_movement();
    update_orientation();

    printf("[ORIENTATION 0010] 4: delta_alpha %d (expected 2826)\r\n", delta_alpha);

    return stat;
}

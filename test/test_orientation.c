#include "test_orientation.h"
#include "orientation.h"
#include "position.h"
#include "settings.h"

extern int32_t test_orientation(void)
{
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

    return (int32_t)delta_alpha;
}

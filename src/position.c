#include "position.h"
#include "orientation.h"
#include "math.h"
#include "settings.h"
#include "coding_wheels.h"

int32_t previous_left_ticks;
int32_t previous_right_ticks;

int32_t delta_left;
int32_t delta_right;

int32_t current_x;
int32_t current_y;

#include "RTT/SEGGER_RTT.h"

extern void compute_movement(void)
{
    delta_left = left_ticks - previous_left_ticks;
    previous_left_ticks = left_ticks;

    delta_right = right_ticks - previous_right_ticks;
    previous_right_ticks = right_ticks;
}

extern void update_position(void)
{
    int32_t d; /* Distance travelled by the robot center, in ticks */
    float delta_alpha; /* Orientation variation, in radian */
    int32_t R; /* Radius, in ticks */

    int32_t x0; /* x coordinate of local circle, in mm */
    int32_t y0; /* y coordinate of local circle, in mm */

    float orientation_f = (float)orientation / ANGLE_MULT_RAD;

    d = (delta_right + delta_left) / 2;
    delta_alpha = (delta_left - delta_right) * 1000.0f / (wheels_gap * ticks_per_m);

    if (0 != delta_alpha) {
        R = (delta_left + delta_right) / (2 * delta_alpha);
        x0 = current_x - ((R * 100) / ticks_per_m) * cos(orientation_f - delta_alpha);
        y0 = current_y - ((R * 100) / ticks_per_m) * sin(orientation_f - delta_alpha);

        current_x = x0 + ((R * 100) / ticks_per_m) * cos(orientation_f);
        current_y = y0 + ((R * 100) / ticks_per_m) * sin(orientation_f);
    } else {
        current_x += ((d * 1000) / ticks_per_m) * cos(orientation_f);
        current_y += ((d * 1000) / ticks_per_m) * sin(orientation_f);
    }

}

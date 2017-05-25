#include "position.h"
#include "orientation.h"
#include "math.h"
#include "settings.h"
#include "coding_wheels.h"

int32_t previous_left_ticks;
int32_t previous_right_ticks;

int32_t delta_left;
int32_t delta_right;

uint32_t current_x;
uint32_t current_y;

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
    int32_t delta_x;
    int32_t delta_y;

    float R;

    float alpha_f;
    float delta_alpha_f;

    //printf("delta_right %d delta_left %d \r\n", delta_right, delta_left);

    if (delta_right == delta_left) {
        delta_x = delta_left * 1000 * cos((float)orientation / ANGLE_MULT) / ticks_per_m;
        delta_y = delta_left * 1000 * sin((float)orientation / ANGLE_MULT) / ticks_per_m;
        //printf ("delta_x %d delta_y %d or %d\r\n", delta_x, delta_y, orientation);
    } else {
        R = (delta_right + delta_left) * 1000 * ANGLE_MULT / (2 * ticks_per_m * delta_alpha);
        //printf("R %d\r\n", R);
        alpha_f = (float)(orientation - delta_alpha) / ANGLE_MULT;
        delta_alpha_f = (float)delta_alpha / ANGLE_MULT;
        delta_x = 2 * R * sin(delta_alpha_f / 2) * cos(alpha_f + delta_alpha_f / 2);
        delta_y = 2 * R * sin(delta_alpha_f / 2) * sin (alpha_f + delta_alpha_f / 2);
    }

    current_x += delta_x;
    current_y += delta_y;

    printf("x %d y %d \r\n", current_x, current_y);
}

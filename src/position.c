#include "position.h"
#include "orientation.h"
#include "math.h"
#include "settings.h"

volatile uint32_t left_ticks;
volatile uint32_t right_ticks;

uint32_t previous_left_ticks;
uint32_t previous_right_ticks;

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

    uint32_t R;

    float theta;
    float beta;

    if (delta_right == delta_left) {
        delta_x = delta_left * 10 * cos((float)orientation / ANGLE_MULT) / ticks_per_cm;
        delta_y = delta_left * 10 * sin((float)orientation / ANGLE_MULT) / ticks_per_cm;
        printf ("delta_x %d delta_y %d or %d\r\n", delta_x, delta_y, orientation);
    } else {
        R = (delta_right + delta_left) * 10 * ANGLE_MULT / (2 * ticks_per_cm * delta_alpha);
        theta = (float)orientation / ANGLE_MULT;
        beta = (delta_alpha - orientation) / (float)ANGLE_MULT;
        delta_x = R * (cos(beta) - cos (theta));
        delta_y = R * (sin(theta) + sin(beta));
    }

    current_x += delta_x;
    current_y += delta_y;
}

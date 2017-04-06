#include "position.h"
#include "orientation.h"
#include "math.h"

volatile uint32_t left_ticks;
volatile uint32_t right_ticks;

uint32_t previous_left_ticks;
uint32_t previous_right_ticks;

int32_t delta_left;
int32_t delta_right;

uint32_t current_x;
uint32_t current_y;


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

    R = (delta_right + delta_left) * delta_alpha / 2;

    if (delta_right == delta_left) {
        delta_x = delta_left * (1 - cos(orientation));
        delta_y = delta_left * sin(orientation);
    } else {
        delta_x = R * (cos(orientation) - cos (delta_alpha - orientation));
        delta_y = R * (sin(orientation) + sin(delta_alpha - orientation));
    }

    current_x += delta_x;
    current_y += delta_y;
}

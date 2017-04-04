#ifndef POSITION_H
#define POSITION_H

extern volatile uint32_t left_ticks;
extern volatile uint32_t right_ticks;

extern uint32_t previous_left_ticks;
extern uint32_t previous_right_ticks;

extern uint32_t delta_left;
extern uint32_t delta_right;

extern uint32_t current_x;
extern uint32_t current_y;

extern void compute_movement(void);
extern void update_position(void);

#endif /* POSITION_H */

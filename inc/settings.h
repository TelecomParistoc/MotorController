#ifndef SETTINGS_H
#define SETTINGS_H

#include "ch.h"

extern uint32_t wheels_gap;
extern uint32_t max_linear_acceleration;
extern uint32_t max_angular_acceleration;
extern uint32_t cruise_linear_speed;
extern uint32_t cruise_angular_speed;
extern uint32_t ticks_per_cm;

extern uint16_t angular_trust_threshold;

#endif /* SETTINGS_H */

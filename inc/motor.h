#ifndef MOTOR_H
#define MOTOR_H

typedef enum {
    MOTOR_LEFT = 0U,
    MOTOR_RIGHT = 1U
} motor_t;

typedef enum {
    FORWARD = 0U,
    BACKWRD = 1U
} motor_direction_t;

extern void init_motor(void);

extern motor_direction_t get_direction(motor_t motor);

extern void set_direction(motor_t motor, motor_direction_t direction);

extern void toggle_direction(motor_t motor);

extern void set_speed(motor_t motor, uint32_t speed);

#endif /* MOTOR_H */

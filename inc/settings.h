#ifndef SETTINGS_H
#define SETTINGS_H

/******************************************************************************/
/*                               Includes                                     */
/******************************************************************************/
#include "ch.h"
#include "coding_wheels.h"
#include "motor.h"

/******************************************************************************/
/*                               Constants                                    */
/******************************************************************************/
/*
 * Datasheet says that this value should be 900 if angles are measured in radians.
 * But the maximum value remains 5760, whatever the unit selected. And
 * 5760 / 2 * pi = 916.73.
 */
#define ANGLE_MULT_RAD 917

#define ANGLE_MULT_DEG 16

/******************************************************************************/
/*                                  Types                                     */
/******************************************************************************/
typedef struct {
    uint16_t p;
    uint16_t i;
    uint16_t d;
} pid_coeffs_t;

typedef struct {
    uint16_t wheels_gap;               /* Distance between the middle of the 2 coding wheels in mm. */
    uint16_t ticks_per_m;              /* Number of coding wheel ticks per m. */
    uint16_t angular_trust_threshold;  /* Threshold for angular speed above which we can't trust the IMU anymore. */
    uint16_t max_linear_acceleration;  /* Maximum linear acceleration authorized, in cm.s-2 */
    uint16_t max_angular_acceleration; /* Maximum angular acceleration authorized, in 1/917 radian.s-2 */
    uint16_t cruise_linear_speed;      /* Target cruise linear speed, in cm.s-1 */
    uint16_t cruise_angular_speed;     /* Target cruise angular speed, in 1/917 radian.s-1 */
    /* PID coeffs. Value will be divided by DIVISION_FACTOR (see control.c) */
    pid_coeffs_t linear_coeff;
    pid_coeffs_t angular_coeff;
    coding_wheels_config_t coding_wheels_config;
    motor_sense_t motor_right_forward_sense;
    motor_sense_t motor_left_forward_sense;
    uint16_t linear_allowance;
    uint16_t angular_allowance;
} robot_settings_t;

/******************************************************************************/
/*                                Variables                                   */
/******************************************************************************/
extern volatile robot_settings_t settings;

#endif /* SETTINGS_H */

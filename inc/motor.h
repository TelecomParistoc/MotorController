#ifndef MOTOR_H
#define MOTOR_H

#include "hal.h"

typedef enum {
    MOTOR_LEFT = 0U,
    MOTOR_RIGHT = 1U
} motor_t;

typedef enum {
    FORWARD = 0U,
    BACKWARD = 1U
} motor_direction_t;

typedef enum {
    DIRECTION_1 = 0U,
    DIRECTION_2 = 1U
} motor_sense_t;

/*
 * @brief Initialise the motor driver.
 *
 * @details This function is in charge of enabling all the low-level components
 *          used to control the motors. It configures the timers used to
 *          generate the PWM and starts them. It also sets the direction pins
 *          so that the motors turn forward as defined by the parameters.
 *
 * @param[in] motor_left_forward_sense The rotation direction which corresponds
 *                                     to a forward movement of the left motor.
 * @param[in] motor_right_forward_sense The rotation direction which corresponds
 *                                     to a forward movement of the right motor.
 *
 * @note To provide a high-level interface to control the motors, this driver
 *       uses the values FORWARD and BACKWARD to describe the rotation direction
 *       of the motors. However, these directions depend on how the motors are
 *       placed in the robot and how they are wired on the H-bridge. In order to
 *       foster reusability of this driver, the mapping between pins state and
 *       rotation direction is settable.
 *       The 2 parameters of this init function are for this purpose.
 *       DIRECTION_1 corresponds to PIN_A at '1' (+3V3) and PIN_B at '0' (GND).
 *       DIRECTION_2 corresponds to PIN_A at '0'  (GND) and PIN_B at '1' (+3V3).
 *
 * @warning This function must be called first to start the motor driver.
 */
extern void motor_init(motor_sense_t motor_left_forward_sense, motor_sense_t motor_right_forward_sense);

/*
 * @brief Set the rotation speed of the specified motor.
 *
 * @param[in] motor The motor to change the speed of.
 * @param[in] speed The requested speed. Allowed range is [0,100]. Be careful
 *                  that a minimum non-zero value is required for the robot to
 *                  move.
 *
 * @return An int indicating success, else an error code.
 * @retval 0 success.
 * @retval -1 Invalid 'motor' parameter.
 * @retval -2 Invalid 'speed' parameter (out of range).
 */
extern int motor_set_speed(motor_t motor, uint32_t speed);

/*
 * @brief Get the current rotation direction of the specified motor.
 *
 * @param[in] motor The motor to look at.
 *
 * @return The current rotation direction of the specified motor.
 * @retval FORWARD/BACKWARD
 * @retval -1 Invalid 'motor' parameter.
 */
extern motor_direction_t motor_get_direction(motor_t motor);

/*
 * @brief Set the rotation direction of the specified motor.
 *
 * @param[in] motor To motor to set the direction of.
 * @param[in] direction The requested rotation direction.
 *
 * @return An int indicating success, else an error code.
 * @retval 0 success.
 * @retval -1 Invalid 'motor' parameter.
 * @retval -2 Invalid 'direction' parameter (out of range).
 */
extern int motor_set_direction(motor_t motor, motor_direction_t direction);

/*
 * @brief Revert the rotation direction of the specified motor.
 *
 * @param[in] motor To motor to change the rotation direction of.
 */
extern void motor_toggle_direction(motor_t motor);

#endif /* MOTOR_H */

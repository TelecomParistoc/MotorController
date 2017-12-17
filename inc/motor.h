/** @file */

#ifndef MOTOR_H
#define MOTOR_H

#include "hal.h"

/** Minimal value that makes the robot move */
#define MIN_COMMAND 1
/** Maximum possible value for commands */
#define MAX_COMMAND 100

/**
 * @brief Alias to select the motor to work on.
 *
 * @remark The meaning of the value "LEFT" and "RIGHT" is not necessarily
 *          coherent with the actual mechanical organization of the robot, it
 *          depends on initial user choice.
 */
typedef enum {
    MOTOR_LEFT = 0U, /**< Left motor */
    MOTOR_RIGHT = 1U /**< Right motor */
} motor_t;

/**
 * @brief Alias for motor rotation sense.
 */
typedef enum {
    FORWARD = 0U, /**< Motor turns so that robot moves forward */
    BACKWARD = 1U /**< Motor turns so that robot moves backward */
} motor_direction_t;

/**
 * @brief Alias for motor orientation.
 *
 * @remark These values allow to hide mechanical differences in motor installation
 *         so that a FORWARD move makes the motor turn in the proper sense.
 *         Values are totally arbitrary.
 */
typedef enum {
    DIRECTION_1 = 0U, /**< Value 1*/
    DIRECTION_2 = 1U /**< Value 2*/
} motor_sense_t;

/**
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

/**
 * @brief Set the rotation speed of the specified motor.
 *
 * @param[in] motor The motor to change the speed of.
 * @param[in] speed The requested speed. Allowed range is [0,MAX_COMMAND]. Be careful
 *                  that a minimum non-zero value (MIN_COMMAND) is required for the robot to
 *                  move.
 *
 * @return An int indicating success, else an error code.
 * @retval 0 Success.
 * @retval -1 Invalid 'motor' parameter.
 * @retval -2 Invalid 'speed' parameter (out of range).
 */
extern int motor_set_speed(motor_t motor, uint8_t speed);

/**
 * @brief Get the rotation speed of the specified motor.
 *
 * @param[in] motor The motor to get the speed of.
 *
 * @return The rotation speed of the specified motor or -1 in case of error.
 */
extern int8_t motor_get_speed(motor_t motor);

/**
 * @brief Get the current rotation direction of the specified motor.
 *
 * @param[in] motor The motor to look at.
 *
 * @return The current rotation direction of the specified motor.
 * @retval FORWARD/BACKWARD
 * @retval -1 Invalid 'motor' parameter.
 */
extern motor_direction_t motor_get_direction(motor_t motor);

/**
 * @brief Set the rotation direction of the specified motor.
 *
 * @param[in] motor To motor to set the direction of.
 * @param[in] direction The requested rotation direction.
 *
 * @return An int indicating success, else an error code.
 * @retval 0 Success.
 * @retval -1 Invalid 'motor' parameter.
 * @retval -2 Invalid 'direction' parameter (out of range).
 */
extern int motor_set_direction(motor_t motor, motor_direction_t direction);

/**
 * @brief Revert the rotation direction of the specified motor.
 *
 * @param[in] motor To motor to change the rotation direction of.
 */
extern void motor_toggle_direction(motor_t motor);

#endif /* MOTOR_H */

#ifndef POSITION_H
#define POSITION_H

/******************************************************************************/
/*                               Includes                                     */
/******************************************************************************/
#include "ch.h"


/******************************************************************************/
/*                                 Types                                      */
/******************************************************************************/
typedef struct {
    int32_t left;
    int32_t right;
} ticks_t;

typedef struct {
    float x;
    float y;
} position_t;

/******************************************************************************/
/*                               Variables                                    */
/******************************************************************************/
/**
 * For tests only.
 */
extern ticks_t previous_ticks;

/**
 * Variation of the coding wheels, in ticks.
 */
extern ticks_t delta_ticks;

/**
 * Current cartesian coordinates of the robot center, in mm.
 */
extern position_t cur_pos;

/******************************************************************************/
/*                          Functions prototypes                              */
/******************************************************************************/
/**
 * @brief Compute the coding wheels movements.
 */
extern void compute_movement(void);

/**
 * @brief Update the position according to the information given by the coding wheels.
 *
 * @details This function must be called AFTER update_orientation(), as it
 *          considers that the 'orientation' variable holds the new orientation.
 */
extern void update_position(void);

#endif /* POSITION_H */

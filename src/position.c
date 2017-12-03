/******************************************************************************/
/*                               Includes                                     */
/******************************************************************************/
#include "position.h"
#include "orientation.h"
#include "math.h"
#include "settings.h"
#include "coding_wheels.h"
#include "RTT/SEGGER_RTT.h"

#include "RTT/SEGGER_RTT.h"

#define ABS(x) (x > 0 ? x : -x)

/******************************************************************************/
/*                           Public variables                                 */
/******************************************************************************/
ticks_t previous_ticks;

ticks_t delta_ticks;

position_t cur_pos;


/*****************************************************************************/
/*                             Local constant                                */
/*****************************************************************************/
#define EPSILON (0.1f)


/******************************************************************************/
/*                             Local macros                                   */
/******************************************************************************/
#define M_TO_MM(x) ((x) / 1000)

/******************************************************************************/
/*                           Public functions                                 */
/******************************************************************************/
extern void compute_movement(void)
{
    delta_ticks.left = left_ticks - previous_ticks.left;
    previous_ticks.left = left_ticks;

    delta_ticks.right = right_ticks - previous_ticks.right;
    previous_ticks.right = right_ticks;
}

extern void update_position(void)
{
    float d; /* Distance travelled by the robot center, in ticks */
    float delta_alpha; /* Orientation variation, in radian */
    float R = 0.0f; /* Radius, in ticks */

    position_t O; /* coordinates of local circle center, in mm */

    float orientation_f = (float)orientation / ANGLE_MULT_RAD;

    /**
     * Coding wheels ticks, at previous cur_pos update.
     * A separate variable is used because cur_pos update frequency can be different
     * from the compute_movement one.
     */
    static ticks_t cur_pos_previous_ticks = {0, 0};

    /* Coding wheels ticks variation since last cur_pos update (last call to this function) */
    ticks_t cur_pos_delta_ticks;

    cur_pos_delta_ticks.right = right_ticks - cur_pos_previous_ticks.right;
    cur_pos_delta_ticks.left = left_ticks - cur_pos_previous_ticks.left;

    cur_pos_previous_ticks.right = right_ticks;
    cur_pos_previous_ticks.left = left_ticks;

    d = (cur_pos_delta_ticks.right + cur_pos_delta_ticks.left) / 2.0f;
    delta_alpha = (cur_pos_delta_ticks.left - cur_pos_delta_ticks.right) * 1000.0f / (settings.wheels_gap * settings.ticks_per_m);

    if (fabs((double) delta_alpha) > EPSILON){ // curve

        R = d / (2 * delta_alpha);
        O.x = cur_pos.x - (R / M_TO_MM(settings.ticks_per_m)) * cos(orientation_f - delta_alpha);
        O.y = cur_pos.y - (R / M_TO_MM(settings.ticks_per_m)) * sin(orientation_f - delta_alpha);

        cur_pos.x = O.x + (R / M_TO_MM(settings.ticks_per_m)) * cos(orientation_f);
        cur_pos.y = O.y + (R / M_TO_MM(settings.ticks_per_m)) * sin(orientation_f);

    } else { // line
        cur_pos.x += (((d * 1000) / settings.ticks_per_m) * cos(orientation_f));
        cur_pos.y += (((d * 1000) / settings.ticks_per_m) * sin(orientation_f));
    }
}

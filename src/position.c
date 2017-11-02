/******************************************************************************/
/*                               Includes                                     */
/******************************************************************************/
#include "position.h"
#include "orientation.h"
#include "math.h"
#include "settings.h"
#include "coding_wheels.h"

#include "RTT/SEGGER_RTT.h"

#define ABS(x) (x > 0 ? x : -x)

/******************************************************************************/
/*                           Public variables                                 */
/******************************************************************************/
ticks_t previous_ticks;

ticks_t delta_ticks;

position_t cur_pos;

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
    int32_t d; /* Distance travelled by the robot center, in ticks */
    float delta_alpha; /* Orientation variation, in radian */
    int32_t R; /* Radius, in ticks */

    position_t O; /* coordinates of local circle center, in mm */

    float orientation_f = (float)orientation / ANGLE_MULT_RAD;

    d = (delta_ticks.right + delta_ticks.left) / 2;
    delta_alpha = (delta_ticks.left - delta_ticks.right) * 1000.0f / (settings.wheels_gap * settings.ticks_per_m);


    static int cpt = 0;
    if (cpt++ % 100 == 0) printf("d = %d; \td_r = %d; d_l = %d\n", d, delta_ticks.right, delta_ticks.left);

    if ( ABS(delta_alpha) > .0001) {
        R = d / delta_alpha;
        O.x = cur_pos.x - ((R * 100) / settings.ticks_per_m) * cos(orientation_f - delta_alpha);
        O.y = cur_pos.y - ((R * 100) / settings.ticks_per_m) * sin(orientation_f - delta_alpha);

        cur_pos.x = O.x + ((R * 100) / settings.ticks_per_m) * cos(orientation_f);
        cur_pos.y = O.y + ((R * 100) / settings.ticks_per_m) * sin(orientation_f);
    } else {
        cur_pos.x += ((d * 1000.) / settings.ticks_per_m) * cos(orientation_f);
        cur_pos.y += ((d * 1000.) / settings.ticks_per_m) * sin(orientation_f);
    }

}

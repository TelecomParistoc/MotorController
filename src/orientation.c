#include "orientation.h"
#include "position.h"
#include "imudriver.h"
#include "tr_types.h"
#include "settings.h"
#include "log.h"

/******************************************************************************/
/*                              Local macros                                  */
/******************************************************************************/
#define PITCH_MIN_VALUE -2880
#define PITCH_RANGE 5760
#define PITCH_MAX_VALUE 2880

#define ROLL_MIN_VALUE -1440
#define ROLL_RANGE 2880
#define ROLL_MAX_VALUE 1440

#define RAD_TO_DEG 57

#define IMU_CWHEELS_RATIO 10.0
#define PI 3.14159

#define ORIENTATION_AVG_COEFF 10

/******************************************************************************/
/*                          Local variables                                   */
/******************************************************************************/
static int16_t pitch_offset = 0;
static int16_t roll_offset = 0;

/******************************************************************************/
/*                            Public variables                                */
/******************************************************************************/
volatile int16_t orientation_changed = -1;
volatile float mixed_orientation = 0;
volatile int16_t heading_offset = 0;
volatile int16_t orientation;
volatile float coding_wheels_orientation;
volatile int16_t IMU_orientation;
float delta_alpha;

/******************************************************************************/
/*                         Public functions                                   */
/******************************************************************************/
extern int32_t set_orientation(int16_t new_orientation)
{
    int32_t status;
	int16_t tmp[5];
	int16_t average;
	uint8_t i;

	if ((new_orientation >= HEADING_MIN_VALUE) && (new_orientation < HEADING_MAX_VALUE)) {
        new_orientation = HEADING_MAX_VALUE - new_orientation; // change back to non-trigonometric orientation
		do {
			average = 0;
			for (i = 0; i < 5; ++i) {
				tmp[i] = getHeading();
				average += tmp[i];
			}
			average /= 5;
		} while (average != tmp[0]);

        heading_offset = average - new_orientation;
		status = NO_ERROR;
    } else {
		status = INVALID_PARAMETER;
	}

  coding_wheels_orientation = (float) new_orientation;

	return status;
}

extern int16_t get_relative_heading(void)
{
    int16_t heading;
    int16_t direction;
    static int16_t prev_direction;

    heading = getHeading();
    if (ANGLE_ERROR == heading) {
        direction = prev_direction;
    } else {
        direction = heading - heading_offset;
        if (direction < HEADING_MIN_VALUE) {
            direction += HEADING_RANGE;
        } else if (direction >= HEADING_MAX_VALUE) {
            direction -= HEADING_RANGE;
        }

        // Change the sense of the angle to have it trigonometric.
        direction = HEADING_MAX_VALUE - direction;
        prev_direction = direction;
    }

    return direction;
}

extern int32_t set_pitch(int16_t pitch) {
	int32_t status;
	int16_t tmp[5];
	int16_t average;
	uint8_t i;

	if ((pitch >= PITCH_MIN_VALUE) && (pitch < PITCH_MAX_VALUE)) {
		do {
			average = 0;
			for (i = 0; i < 5; i++) {
				tmp[i] = getPitch();
				average += tmp[i];
			}
			average /= 5;
		} while (average != tmp[0]);

        pitch_offset = average - pitch;
		status = NO_ERROR;
    } else {
		status = INVALID_PARAMETER;
	}

	return status;
}

extern int16_t get_relative_pitch(void) {
    int16_t pitch;

    pitch = getPitch();
    if (pitch != ANGLE_ERROR) {
        pitch -= pitch_offset;
        if (pitch < PITCH_MIN_VALUE) {
            pitch += PITCH_RANGE;
        }
    }

    return pitch;
}


extern int32_t set_roll(int16_t roll) {
	int32_t status;
	int16_t tmp[5];
	int16_t average;
	uint8_t i;

	if ((roll >= ROLL_MIN_VALUE) && (roll < ROLL_MAX_VALUE)) {
		do {
			average = 0;
			for (i = 0; i < 5; i++) {
				tmp[i] = getRoll();
				average += tmp[i];
			}
			average /= 5;
		} while (average != tmp[0]);

        roll_offset = average - roll;
		status = NO_ERROR;
    } else {
		status = INVALID_PARAMETER;
	}

	return status;
}

extern int16_t get_relative_roll(void) {
    int16_t roll;

    roll = getRoll();
    if (roll != ANGLE_ERROR) {
        roll -= roll_offset;
        if (roll < ROLL_MIN_VALUE) {
            roll += ROLL_RANGE;
        }
    }

    return roll;
}

extern void update_orientation(void)
{
    static uint32_t prev_time = 0U;
    uint32_t cur_time;

    cur_time = chVTGetSystemTime();

    /* checks if a new orientation must be set */
    /* orientation_changed != -1 means such an order comes from i2c_interface */
    /* in this case orientation_changed contains the new value */
    if (orientation_changed != -1){
      heading_offset = orientation_changed + heading_offset - orientation;
      while (heading_offset >= HEADING_MAX_VALUE)
        heading_offset -= HEADING_RANGE;
      while (heading_offset < HEADING_MIN_VALUE)
        heading_offset += HEADING_RANGE;

      IMU_orientation = orientation_changed;
      coding_wheels_orientation = (float) orientation_changed;
      orientation = orientation_changed;
      mixed_orientation = orientation_changed;

      orientation_changed = -1;
    }
    else if (0U != prev_time) {

        /* Compute delta alpha in radian */
        //settings.wheels_gap is in mm
        delta_alpha = (float)(delta_ticks.right - delta_ticks.left) * 500. / (settings.wheels_gap * settings.ticks_per_m);

        coding_wheels_orientation += delta_alpha * ANGLE_MULT_RAD;
        if (coding_wheels_orientation < 0) {
            coding_wheels_orientation += HEADING_MAX_VALUE;
        }
        else if (coding_wheels_orientation > HEADING_MAX_VALUE){
          coding_wheels_orientation -= HEADING_MAX_VALUE;
        }

        IMU_orientation = get_relative_heading();

        /* computes the most probable orientation using an absolute information
         * (IMU_orientation) and a relative information (delta_alpha) */
        float theta_1 = IMU_orientation;
        float theta_2 = mixed_orientation + delta_alpha * ANGLE_MULT_RAD;
        if (theta_2 - theta_1 >= HEADING_MAX_VALUE / 2) theta_2 -= HEADING_MAX_VALUE;
        if (theta_1 - theta_2 >= HEADING_MAX_VALUE / 2) theta_2 += HEADING_MAX_VALUE;
        mixed_orientation = (theta_1 + IMU_CWHEELS_RATIO * theta_2) / (1. + IMU_CWHEELS_RATIO);
        if (mixed_orientation < 0) mixed_orientation += HEADING_MAX_VALUE;
        if (mixed_orientation >= HEADING_MAX_VALUE) mixed_orientation -= HEADING_MAX_VALUE;

        orientation = (int16_t) mixed_orientation;

    } else {
        /* First call to this function */
        orientation = get_relative_heading();
        IMU_orientation = orientation;
        coding_wheels_orientation = (float) orientation;
        mixed_orientation = (float) orientation;
    }

    prev_time = cur_time;
}

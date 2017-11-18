#include "orientation.h"
#include "position.h"
#include "imudriver.h"
#include "tr_types.h"
#include "settings.h"

#include "RTT/SEGGER_RTT.h"

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

/******************************************************************************/
/*                          Local variables                                   */
/******************************************************************************/
static int16_t pitch_offset = 0;
static int16_t roll_offset = 0;

/******************************************************************************/
/*                            Public variables                                */
/******************************************************************************/
int16_t heading_offset = 0;
int16_t orientation;
int16_t delta_alpha;

/******************************************************************************/
/*                         Public functions                                   */
/******************************************************************************/
extern int32_t set_heading(int16_t heading)
{
    int32_t status;
	int16_t tmp[5];
	int16_t average;
	uint8_t i;

	if ((heading >= HEADING_MIN_VALUE) && (heading < HEADING_MAX_VALUE)) {
		do {
			average = 0;
			for (i = 0; i < 5; ++i) {
				tmp[i] = getHeading();
				average += tmp[i];
			}
			average /= 5;
		} while (average != tmp[0]);

        heading_offset = getHeading() - heading;
		status = NO_ERROR;
    } else {
		status = INVALID_PARAMETER;
	}

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
        } else if (direction > HEADING_MAX_VALUE) {
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

        pitch_offset = getPitch() - pitch;
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

        roll_offset = getRoll() - roll;
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
    uint32_t delta_time;
    int16_t tmp_speed;

    cur_time = chVTGetSystemTime();

    if (0U != prev_time) {
        delta_time = cur_time - prev_time;

        /* Compute delta alpha in radian */
        delta_alpha = (int16_t)(delta_ticks.right - delta_ticks.left) * 100 / (settings.wheels_gap * settings.ticks_per_m);

        /* Compute local angular speed in °.s-1 */
        tmp_speed = delta_alpha * RAD_TO_DEG / ST2S(delta_time);

        //printf("tmp_speed = %d; seuil = %d\n", tmp_speed, settings.angular_trust_threshold);

        /* If variation is fast, don't use the IMU */
        if (0 && ((tmp_speed <= -settings.angular_trust_threshold) || (tmp_speed >= settings.angular_trust_threshold))) {
            orientation += delta_alpha * ANGLE_MULT_RAD;

            if (orientation < 0) {
                orientation += HEADING_MAX_VALUE;
            }

            orientation %= HEADING_MAX_VALUE;
        } else { /* Small variation, use IMU as it's more precise */
            orientation = get_relative_heading();
        }
    } else {
        /* First call to this function */
        orientation = get_relative_heading();
    }

    prev_time = cur_time;
}

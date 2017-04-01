#include "position.h"
#include "imudriver.h"
#include "tr_types.h"

/******************************************************************************/
/*                              Local macros                                  */
/******************************************************************************/
#define HEADING_MIN_VALUE 0
#define HEADING_RANGE 5760
#define HEADING_MAX_VALUE 5760

#define PITCH_MIN_VALUE -2880
#define PITCH_RANGE 5760
#define PITCH_MAX_VALUE 2880

#define ROLL_MIN_VALUE -1440
#define ROLL_RANGE 2880
#define ROLL_MAX_VALUE 1440

/******************************************************************************/
/*                          Local variables                                   */
/******************************************************************************/
static int16_t heading_offset = 0;
static int16_t pitch_offset = 0;
static int16_t roll_offset = 0;

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

extern int16_t get_direction(void)
{
    int16_t heading;
    int16_t direction;

    heading = getHeading();
    if (heading == ANGLE_ERROR) {
        direction = ANGLE_ERROR;
    } else {
        direction = heading - heading_offset;
        if (direction < HEADING_MIN_VALUE) {
            direction += HEADING_RANGE;
        }
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

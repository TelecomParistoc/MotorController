#include "ch.h"
#include "hal.h"
#include "imudriver.h"
#include "tr_types.h"

#include "RTT/SEGGER_RTT.h"

static const I2CConfig i2c2_cfg = {
	0x20420F13,
	0x00000001,
	0
};

int main(void) {
	// initialize ChibiOS
	halInit();
	chSysInit();

	// initialize hardware
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	//printf("J'suis dans les air t'es dans les bouchons.\n");

	volatile int status;
	volatile int16_t heading;
	i2cStart(&I2CD2, &i2c2_cfg);
	status = initIMU(&I2CD2);
	if (status == NO_ERROR) {
		printf("Init OK\n");
	} else {
		printf("Error in IMU init\n");
	}

	setHeading(0);

	while(TRUE) {
		chThdSleepMilliseconds(100);
		heading = getHeading();
		printf("heading : %u --\n", heading);
		palTogglePad(GPIOA, GPIOA_RUN_LED);
	}


	chThdSleep(TIME_INFINITE);
	return 0;
}

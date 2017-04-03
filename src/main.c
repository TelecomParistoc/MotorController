#include "ch.h"
#include "hal.h"
#include "imudriver.h"
#include "tr_types.h"
#include "position.h"

#include "RTT/SEGGER_RTT.h"

int main(void) {
	// initialize ChibiOS
	halInit();
	chSysInit();

	// initialize hardware
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	volatile int status;
	volatile int16_t heading;
	int i;

	i2cStart(&I2CD2, &imu_i2c_conf);

	status = initIMU(&I2CD2);
	if (status == NO_ERROR) {
		printf("Init OK\n");
	} else {
		printf("Error in IMU init\n");
	}

	for (i = 0; i < 100; ++i) {
		chThdSleepMilliseconds(100);
		printf("dir %u\r\n", get_relative_roll());
	}

	set_roll(0);

	while(TRUE) {
		chThdSleepMilliseconds(100);
		heading = get_relative_roll();
		printf("roll : %u (raw %u)--\n", heading, getRoll());
		palTogglePad(GPIOA, GPIOA_RUN_LED);
	}


	chThdSleep(TIME_INFINITE);
	return 0;
}

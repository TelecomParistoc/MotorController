#include "ch.h"
#include "hal.h"
#include "imudriver.h"
#include "tr_types.h"
#include "position.h"
#include "test_position.h"
#include "test_orientation.h"
#include "test.h"

#include "RTT/SEGGER_RTT.h"

int main(void) {
	// initialize ChibiOS
	halInit();
	chSysInit();

	// initialize hardware
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	volatile int status;
	int32_t ret_value;

	i2cStart(&I2CD2, &imu_i2c_conf);

	status = initIMU(&I2CD2);
	if (status == NO_ERROR) {
		printf("Init OK\n");
	} else {
		printf("Error in IMU init\n");
	}

	ret_value = test_position_0010();
	if (ret_value == TEST_NO_ERROR) {
		printf("test position 0010 succeeded \r\n");
	} else {
		printf("test position 0010 failed %u \r\n", ret_value);
	}

	ret_value = test_orientation();
	printf("test orientation %u \r\n");


	while(TRUE) {
		chThdSleepMilliseconds(100);
		palTogglePad(GPIOA, GPIOA_RUN_LED);
	}


	chThdSleep(TIME_INFINITE);
	return 0;
}

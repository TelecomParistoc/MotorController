#include "ch.h"
#include "hal.h"
#include "imudriver.h"
#include "tr_types.h"
#include "position.h"
#include "orientation.h"
#include "i2c_interface.h"
#include "coding_wheels.h"
#include "motor.h"
#include "log.h"
#include "control.h"
#include "settings.h"
#include "data_storage.h"

int main(void) {
	int status;

	// initialize ChibiOS
	halInit();
	chSysInit();

	/* Let some time to the IMU for reset */
	chThdSleepMilliseconds(2000);

	// initialize hardware
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	i2cStart(&I2CD2, &imu_i2c_conf);

	status = initIMU(&I2CD2);
	if (status == NO_ERROR) {
		LOG_VERBOSE("Init OK\n");
	} else {
		LOG_VERBOSE("Error in IMU init\n");
	}

	setFormat(RADIAN);

	i2c_slave_init(&I2CD1);

	load_data_from_flash();
	init_coding_wheels(settings.coding_wheels_config);
	motor_init(settings.motor_left_forward_sense, settings.motor_right_forward_sense);

	chThdCreateStatic(wa_control, sizeof(wa_control), NORMALPRIO + 1, control_thread, NULL);
	chThdCreateStatic(wa_int_pos, sizeof(wa_int_pos), NORMALPRIO + 1, int_pos_thread, NULL);
	chThdCreateStatic(wa_reset_pos, sizeof(wa_reset_pos), NORMALPRIO + 1, reset_pos_thread, NULL);

	while(TRUE) {
		chThdSleepMilliseconds(50);
		palTogglePad(GPIOA, GPIOA_RUN_LED);
	}
	chThdSleep(TIME_INFINITE);
	return 0;
}

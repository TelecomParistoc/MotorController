#include "ch.h"
#include "hal.h"
#include "imudriver.h"
#include "tr_types.h"
#include "position.h"
#include "orientation.h"
#include "test.h"
#include "i2c_interface.h"
#include "coding_wheels.h"
#include "motor.h"
#include "RTT/SEGGER_RTT.h"
#include "control.h"
#include "settings.h"
#include "config.h"

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

	coding_wheels_config_t cod_cfg = {
		0U,
		DIRECT,
		0U,
		INDIRECT
	};
	init_coding_wheels(cod_cfg);
	motor_init(DIRECTION_2, DIRECTION_2);

	status = initIMU(&I2CD2);
	if (status == NO_ERROR) {
		printf("Init OK\n");
	} else {
		printf("Error in IMU init\n");
	}

	setFormat(RADIAN);

	i2c_slave_init(&I2CD1);

	chThdCreateStatic(wa_control, sizeof(wa_control), NORMALPRIO + 1, control_thread, NULL);
	chThdCreateStatic(wa_int_pos, sizeof(wa_int_pos), NORMALPRIO + 1, int_pos_thread, NULL);
#ifdef BIG
	max_linear_acceleration = 800;
	max_angular_acceleration = 3000;

	linear_p_coeff = 600;
	linear_i_coeff = 2;
	//linear_d_coeff = 5000;

	angular_p_coeff = 200;
	angular_i_coeff = 10;
	//angular_d_coeff = 30;

	goal_mean_dist = -800; /* in mm */
	//goal_heading = 2879;

	heading_dist_sync_ref = 0;
	ticks_per_m = 5100;
	wheels_gap = 150;

	cruise_linear_speed = 5000;
	cruise_angular_speed = 50000;
	angular_trust_threshold = 100;

	dist_command_received = TRUE;
#else /* SMALL */
	printf("foo\r\n");
#endif /* BIG */

	while(TRUE) {
		chThdSleepMilliseconds(50);
		palTogglePad(GPIOA, GPIOA_RUN_LED);
		//printf("------------- ticks %d || %d\r\n", left_ticks, right_ticks);
		//printf("heading %d\r\n", orientation);
		if (msg_received) {
			printf("=================================\r\n");
			printf("%d\r\n", msg);
			printf("(%d)\r\n", current_x);
			msg_received = FALSE;
		}
	}

	chThdSleep(TIME_INFINITE);
	return 0;
}

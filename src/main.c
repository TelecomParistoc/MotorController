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

int32_t time = 0;

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
		printf("Init OK\n");
	} else {
		printf("Error in IMU init\n");
	}

	setFormat(RADIAN);

	i2c_slave_init(&I2CD1);

#ifdef BIG
	/* Max accelerations */
	max_linear_acceleration = 400;
	max_angular_acceleration = 800;

	/* Linear PID coeffs */
	linear_p_coeff = 400;
	linear_i_coeff = 3;
	//linear_d_coeff = 5000;

	/* Angular PID coeffs */
	angular_p_coeff = 250;
	angular_i_coeff = 3;
	//angular_d_coeff = 30;

	/* Initial goals */
	goal_mean_dist = 0; /* in mm */
	goal_heading = 0;

	/* config */
	heading_dist_sync_ref = 0;
	ticks_per_m = 5100;
	wheels_gap = 150;

	/* linear speed */
	cruise_linear_speed = 300;
	cruise_angular_speed = 1500;
	angular_trust_threshold = 100;

	dist_command_received = TRUE;

	/* Init motors */
	coding_wheels_config_t cod_cfg = {
		0U,
		DIRECT,
		0U,
		INDIRECT
	};
	init_coding_wheels(cod_cfg);
	motor_init(DIRECTION_2, DIRECTION_2);

#else /* SMALL */
	/* Max accelerations */
	max_linear_acceleration = 50;
	max_angular_acceleration = 400;

	/* Linear PID coeffs */
	linear_p_coeff = 100;
	linear_i_coeff = 2;
	//linear_d_coeff = 0;

	/* Angular PID coeffs */
	angular_p_coeff = 100;
	angular_i_coeff = 2;
	//angular_d_coeff = 0;

	/* Initial goals */
	goal_mean_dist = 0;
	goal_heading = 0;

	/* config */
	heading_dist_sync_ref = 0;
	wheels_gap = 195;
	ticks_per_m = 5100;

	/* linear speed */
	cruise_linear_speed = 200;
	cruise_angular_speed = 800;
	angular_trust_threshold = 100;
	dist_command_received = TRUE;

	/* Init motors */
	coding_wheels_config_t cod_cfg = {
		0U,
		INDIRECT,
		0U,
		DIRECT
	};
	init_coding_wheels(cod_cfg);
	motor_init(DIRECTION_1, DIRECTION_2);
#endif /* BIG */

chThdCreateStatic(wa_control, sizeof(wa_control), NORMALPRIO + 1, control_thread, NULL);
chThdCreateStatic(wa_int_pos, sizeof(wa_int_pos), NORMALPRIO + 1, int_pos_thread, NULL);

	while(TRUE) {
		chThdSleepMilliseconds(50);
		time += 50;
		palTogglePad(GPIOA, GPIOA_RUN_LED);
		//printf("------------- ticks %d || %d\r\n", left_ticks, right_ticks);
		//printf("heading %d (off %d)\r\n", orientation, heading_offset);
		if (msg_received) {
			printf("msg: %d\r\n", msg);
			msg_received = FALSE;
		}

	}

	chThdSleep(TIME_INFINITE);
	return 0;
}

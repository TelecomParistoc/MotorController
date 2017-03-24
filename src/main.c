#include "ch.h"
#include "hal.h"

#include "RTT/SEGGER_RTT.h"

int main(void) {
	// initialize ChibiOS
	halInit();
	chSysInit();

	// initialize hardware
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	printf("J'suis dans les air t'es dans les bouchons.\n");

	chThdSleep(TIME_INFINITE);
	return 0;
}

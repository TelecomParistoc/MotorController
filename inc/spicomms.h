#ifndef SPICOMMS_H
#define SPICOMMS_H

#include "ch.h"

extern volatile int calibration;
extern volatile int reposition;

extern uint16_t xRadio;
extern uint16_t yRadio;
extern uint16_t xSmallFoe;
extern uint16_t ySmallFoe;
extern uint16_t xBigFoe;
extern uint16_t yBigFoe;


extern uint16_t xRepos;
extern uint16_t yRepos;

/* start SPI thread */
void startSPI(void);

#endif

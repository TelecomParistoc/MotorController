#include "hal.h"

#include "spicomms.h"

#define CALIBRATION_SYMBOL 0b1101
#define REPOSITION_SYMBOL 0b1011

#define TL_PACKET_SIZE_MOSI 6
#define TL_PACKET_SIZE_MISO 12

/*
 * SPI TX and RX buffers.
 */
static uint8_t mosiBuff[TL_PACKET_SIZE_MOSI];
static uint8_t misoBuff[TL_PACKET_SIZE_MISO];

int volatile calibration = 0;
int volatile reposition = 0;

uint16_t xRadio;
uint16_t yRadio;
uint16_t xSmallFoe;
uint16_t ySmallFoe;
uint16_t xBigFoe;
uint16_t yBigFoe;

uint16_t xRepos = 0;
uint16_t yRepos = 0;

static const SPIConfig spi3_master = {
   .end_cb     = NULL,
   .ssport     = GPIOB,
   .sspad      = GPIOA_RAD_N_CS,
   .cr1        = SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0, // 140kHz
   .cr2        = 0
};

static THD_WORKING_AREA(waSPI, 256);
static THD_FUNCTION(spi_thread, p) {

  (void)p;

  chRegSetThreadName("SPI thread master");

  spiStart(&SPID3, &spi3_master);

  while (true) {
    spiSelect(&SPID3);
    mosiBuff[0] = 230;  // synchronization byte
    mosiBuff[1] = 0; // calibration and reposition byte
    for (int i = 0; i < 4; i++) { // coordinates bytes
      mosiBuff[i + 2] = 0;
    }

    if (calibration == 1) {
      mosiBuff[1] &= 0xf0;
      mosiBuff[1] |= CALIBRATION_SYMBOL;
      calibration = 0;
    }
    if (reposition) {
      mosiBuff[1] &= 0x0f;
      mosiBuff[1] |= (REPOSITION_SYMBOL << 4);
      mosiBuff[2] = xRepos;
      mosiBuff[3] = xRepos >> 8;
      mosiBuff[2] = yRepos;
      mosiBuff[3] = yRepos >> 8;
      reposition = 0;
    }

    spiSend(&SPID3, TL_PACKET_SIZE_MOSI, mosiBuff);
    spiReceive(&SPID3, TL_PACKET_SIZE_MISO, misoBuff);

    xRadio = (uint16_t) misoBuff[0];
    yRadio = (uint16_t) misoBuff[2];
    xSmallFoe = (uint16_t) misoBuff[4];
    ySmallFoe = (uint16_t) misoBuff[6];
    xBigFoe = (uint16_t) misoBuff[8];
    yBigFoe = (uint16_t) misoBuff[10];

    spiUnselect(&SPID3);
    chThdSleepMilliseconds(10);
  }
}

void startSPI(void) {
  chThdCreateStatic(waSPI, sizeof(waSPI), NORMALPRIO+1, spi_thread, NULL);
}

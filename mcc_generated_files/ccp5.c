

/**
  Section: Included Files
 */

#include <xc.h>
#include "ccp5.h"
#include "pin_manager.h"

/**
  Section: Capture Module APIs
 */
volatile CCP_PERIOD_REG_T moduleR;
volatile uint8_t isValidR = 0;
volatile uint8_t stoppedR = 0;
volatile uint16_t ticksR = 0;
volatile int8_t ticksHR = 0;
volatile uint8_t directionR = 0;

void CCP5_Initialize(void) {
    // Set the CCP5 to the options selected in the User Interface

    // DC5B LSBs; CCP5M capture_rising edge; 
    CCP5CON = 0x05;

    // CCPR5L 0x0; 
    CCPR5L = 0x00;

    // CCPR5H 0x0; 
    CCPR5H = 0x00;

    // Clear the CCP5 interrupt flag
    PIR4bits.CCP5IF = 0;

    // Enable the CCP5 interrupt
    PIE4bits.CCP5IE = 1;

    // Selecting Timer5
    CCPTMRS1bits.C5TSEL = 0x2;
}

void CCP5_CaptureISR(void) {
    // reset TMR5
    TMR5 = 0;
    // Clear the CCP5 interrupt flag
    PIR4bits.CCP5IF = 0;
    // retrieve current direction
    directionR = RCODER_GetValue();

    if(isValidR) {
        stoppedR = 0;
    // Copy captured value.
        moduleR.ccpr5l = CCPR5L;
        moduleR.ccpr5h = CCPR5H;
        if(directionR) {
            ticksR++;
            if(!ticksR)
                ticksHR++;
        } else {
            if(!ticksR)
                ticksHR--;
            ticksR--;
}
    } else
        isValidR = 1;
}

int16_t getRticks() {
    
    return (ticksR>>3)+(ticksHR<<13);
}
int16_t getRperiod() {
    uint16_t period = moduleR.ccpr5_16Bit;
    if(stoppedR || period > 0x7FFF) {
        return 0x7FFF;
    } else if(directionR) {
        return period;
    } else {
        return -period;
    }
}
int16_t getRspeed() {
    uint16_t period = moduleR.ccpr5_16Bit;
    if(stoppedR || period > 0x7FFF) {
        return 0;
    } else if(directionR) {
        return 32766/period;
    } else {
        return -((int) 32766/period);
    }
}
int16_t getRspeedINT() {
    uint16_t period = moduleR.ccpr5_16Bit;
    if(stoppedR || period > 0x7FFF) {
        return 0;
    } else if(directionR) {
        return 32766/period;
    } else {
        return -((int) 32766/period);
    }
}
void setRticks(int16_t value) {
    ticksR = value << 3;
    ticksHR = value >> 13;
}
/**
 End of File
 */
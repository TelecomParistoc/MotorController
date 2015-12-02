/**
  Section: Included Files
 */

#include <xc.h>
#include "ccp4.h"
#include "pin_manager.h"
#include <stdio.h>

volatile CCP_PERIOD_REG_T module;
volatile uint8_t isValidL = 0;
volatile uint8_t stoppedL = 0;
volatile uint16_t ticks = 0;
volatile int8_t ticksH = 0;
volatile uint8_t direction = 0;
/**
  Section: Capture Module APIs
 */

void CCP4_Initialize(void) {
    // Set the CCP4 to the options selected in the User Interface

    // DC4B LSBs; CCP4M capture_rising edge; 
    CCP4CON = 0x05;

    // CCPR4L 0x0; 
    CCPR4L = 0x00;

    // CCPR4H 0x0; 
    CCPR4H = 0x00;

    // Clear the CCP4 interrupt flag
    PIR4bits.CCP4IF = 0;

    // Enable the CCP4 interrupt
    PIE4bits.CCP4IE = 1;

    // Selecting Timer1
    CCPTMRS1bits.C4TSEL = 0x0;
}

void CCP4_CaptureISR(void) {
    // reset TMR1
    TMR1 = 0;
    // Clear the CCP4 interrupt flag
    PIR4bits.CCP4IF = 0;
    // retrieve current direction
    direction = LCODER_GetValue();
    
    if(isValidL) {
        stoppedL = 0;
        // Copy captured value.
        module.ccpr4l = CCPR4L;
        module.ccpr4h = CCPR4H;
        if(direction) {
            ticks++;
            if(!ticks)
                ticksH++;
        } else {
            if(!ticks)
                ticksH--;
            ticks--;
        }            
    } else
        isValidL = 1;
    
}

int16_t getLticks() {
    return (ticks>>3)+(ticksH<<13);
}
void setLticks(int16_t value) {
    ticks = value << 3;
    ticksH = value >> 13;
}
int16_t getLperiod() {
    uint16_t period = module.ccpr4_16Bit;
    if(stoppedL || period > 0x7FFF) {
        return 0x7FFF;
    } else if(direction) {
        return period;
    } else {
        return -period;
    }
}
int16_t getLspeed() {
    uint16_t period = module.ccpr4_16Bit;
    if(stoppedL || period > 0x7FFF) {
        return 0;
    } else if(direction) {
        return 32766/period;
    } else {
        return -((int) 32766/period);
    }
}
int16_t getLspeedINT() {
    uint16_t period = module.ccpr4_16Bit;
    if(stoppedL || period > 0x7FFF) {
        return 0;
    } else if(direction) {
        return 32766/period;
    } else {
        return -((int) 32766/period);
    }
}
/**
 End of File
 */
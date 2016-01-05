

/**
  Section: Included Files
 */

#include <xc.h>
#include "tmr5.h"
#include "ccp5.h"
#include "../motors.h"

/**
  Section: Global Variable Definitions
 */
volatile uint16_t timer5ReloadVal;

/**
  Section: TMR5 APIs
 */

void TMR5_Initialize(void) {
    //Set the Timer to the options selected in the GUI

    //T5SYNC do_not_synchronize; T5SOSCEN disabled; TMR5CS FOSC/4; TMR5ON disabled; T5RD16 disabled; T5CKPS 1:4; 
    T5CON = 0x24;

    //T5GPOL low; T5GSS T5G; T5GGO done; TMR5GE disabled; T5GSPM disabled; T5GTM disabled; T5GVAL disabled; 
    T5GCON = 0x00;

    //TMR5H 0; 
    TMR5H = 0x00;

    //TMR5L 0; 
    TMR5L = 0x00;

    // Load the TMR value to reload variable
    timer5ReloadVal = TMR5;

    // Clearing IF flag before enabling the interrupt.
    PIR5bits.TMR5IF = 0;

    // Enabling TMR5 interrupt.
    PIE5bits.TMR5IE = 1;

    // Start TMR5
    TMR5_StartTimer();
}

void TMR5_StartTimer(void) {
    // Start the Timer by writing to TMRxON bit
    T5CONbits.TMR5ON = 1;
}

void TMR5_StopTimer(void) {
    // Stop the Timer by writing to TMRxON bit
    T5CONbits.TMR5ON = 0;
}

uint16_t TMR5_ReadTimer(void) {
    uint16_t readVal;
    uint8_t readValHigh;
    uint8_t readValLow;

    readValLow = TMR5L;
    readValHigh = TMR5H;

    readVal = ((uint16_t) readValHigh << 8) | readValLow;

    return readVal;
}

void TMR5_WriteTimer(uint16_t timerVal) {
    if (T5CONbits.T5SYNC == 1) {
        // Stop the Timer by writing to TMRxON bit
        T5CONbits.TMR5ON = 0;

        // Write to the Timer5 register
        TMR5H = (timerVal >> 8);
        TMR5L = (uint8_t) timerVal;

        // Start the Timer after writing to the register
        T5CONbits.TMR5ON = 1;
    } else {
        // Write to the Timer5 register
        TMR5H = (timerVal >> 8);
        TMR5L = (uint8_t) timerVal;
    }
}

void TMR5_Reload(void) {
    // Write to the Timer5 register
    TMR5H = (timer5ReloadVal >> 8);
    TMR5L = (uint8_t) timer5ReloadVal;
}

void TMR5_StartSinglePulseAcquisition(void) {
    T5GCONbits.T5GGO = 1;
}

uint8_t TMR5_CheckGateValueStatus(void) {
    return T5GCONbits.T5GVAL;
}

void TMR5_ISR(void) {
    // Clear the TMR5 interrupt flag
    PIR5bits.TMR5IF = 0;

    // Write to the Timer3 register
    TMR5H = 0;
    TMR5L = 0;
    
    // Add your TMR1 interrupt custom code
    isValidR = 0;
    stoppedR = 1;
}


/**
 End of File
 */

#include <stdio.h>
#include "mcc_generated_files/mcc.h"
#include "motors.h"
#include "mcc_generated_files/ccp4.h"
#include "mcc_generated_files/ccp5.h"
#include "PIDcompute.h"
#include "i2c-interface.h"
/*
                         Main application
 */
void main(void) {
    // Initialize the device
    SYSTEM_Initialize();
    
    INTERRUPT_GlobalInterruptHighEnable();
    INTERRUPT_GlobalInterruptLowEnable();
    
    configureI2Cinterface();
    PIDloadCoeffs();
    
    setMotorL(0);
    setMotorR(0);

    while (1) {
        PIDmanager();
//        if(getLticks() > 128) {
//            setTargetLspeed(0);
//            setTargetRspeed(0);
//        }
        //printf("speed : %d\n", getRspeed());
    }
}
/**
 End of File
 */
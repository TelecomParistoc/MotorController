#include "motors.h"

#include "mcc_generated_files/epwm1.h"
#include "mcc_generated_files/epwm2.h"
#include "mcc_generated_files/pin_manager.h"

#include "PIDcompute.h"

uint8_t stalled = 0;

void setMotorL(int16_t speed) {
    if(speed<THRESHOLD && speed >- THRESHOLD) {
        IN1B_SetLow();
        IN2B_SetLow();
        EPWM1_LoadDutyValue(0);
    } else if(speed>0) {
        if(speed>1023)
            speed = 1023;
        IN2B_SetLow();
        IN1B_SetHigh();
        EPWM1_LoadDutyValue(speed);
    } else {
        IN1B_SetLow();
        IN2B_SetHigh();
        if(speed<-1023)
            speed = -1023;
        EPWM1_LoadDutyValue((-speed));
    }
}

void setMotorR(int16_t speed) {
    if(speed<THRESHOLD && speed >- THRESHOLD) {
        IN1A_SetLow();
        IN2A_SetLow();
        EPWM2_LoadDutyValue(0);
    } else if(speed<0) {
        if(speed<-1023)
            speed = -1023;
        IN1A_SetLow();
        IN2A_SetHigh();
        EPWM2_LoadDutyValue(-speed);
    } else {
        if(speed>1023)
            speed = 1023;
        IN2A_SetLow();
        IN1A_SetHigh();
        EPWM2_LoadDutyValue(speed);
    }
}

void setMotorLi2c(int16_t speed) {
    setMotorL(speed);
    enablePID(0); // disable PID 
}
void setMotorRi2c(int16_t speed) {
    setMotorR(speed);
    enablePID(0); // disable PID 
}



// call whenever the robot is stopped : check it's not trying to move
void stallDetectL() {
    if(CCPR1L & 0xE0) {
        stalled = 1;
        INTOUT_SetHigh();
        INTOUT_SetDigitalOutput();
    }
}

void stallDetectR() {
    if(CCPR2L & 0xE0) {
        stalled = 1;
        INTOUT_SetHigh();
        INTOUT_SetDigitalOutput();
    }
}
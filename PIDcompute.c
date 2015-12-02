#include "PIDcompute.h"

#include "mcc_generated_files/ccp4.h"
#include "mcc_generated_files/ccp5.h"
#include "mcc_generated_files/memory.h"

#include "motors.h"
#include <stdio.h>

uint8_t enabled = 1; // PID enable flag
int8_t Ki = 1;
int8_t Kp = 20;
int8_t Kd = 5;
int16_t targetR = 0;
int16_t targetL = 0;
int16_t integralR = 0;
int16_t integralL = 0;
int16_t lastErrorR = 0;
int16_t lastErrorL = 0;
volatile int16_t modifiedK = 0;

void PIDloadCoeffs() {
    Kp = DATAEE_ReadByte(EE_Kp_ADDR);
    Ki = DATAEE_ReadByte(EE_Ki_ADDR);
    Kd = DATAEE_ReadByte(EE_Kd_ADDR);
}

void PIDmanager() {
    if(enabled) {
        setMotorR(computePID(getRspeed(),targetR, &integralR, &lastErrorR) >> 5);
        setMotorL(computePID(getLspeed(),targetL, &integralL, &lastErrorL) >> 5);
    }
    
    // save PID coefficients in EEPROM if they have been modified
    if(modifiedK & MODIFIED_KP_MASK) {
        DATAEE_WriteByte(EE_Kp_ADDR, Kp);
        modifiedK = modifiedK ^ MODIFIED_KP_MASK; // clear Kp flag bit
    }
    if(modifiedK & MODIFIED_KI_MASK) {
        DATAEE_WriteByte(EE_Ki_ADDR, Ki);
        modifiedK = modifiedK ^ MODIFIED_KI_MASK; // clear Ki flag bit
    }
    if(modifiedK & MODIFIED_KD_MASK) {
        DATAEE_WriteByte(EE_Kd_ADDR, Kd);
        modifiedK = modifiedK ^ MODIFIED_KD_MASK; // clear Kd flag bit
    }
}

int16_t computePID(int16_t speed, int16_t targetSpeed, int16_t * integral, int16_t * lastError) {
    int16_t error = targetSpeed - speed;
    int16_t derivative = error - *lastError;
    *lastError = error;
    *integral += error*Ki;
    if(*integral>MAX_INTEGRAL)
        *integral=MAX_INTEGRAL;
    else if(*integral<MIN_INTEGRAL)
        *integral=MIN_INTEGRAL;
    return Kp*error + *integral - Kd*derivative;
}

void setTargetRspeed(int16_t target) {
    targetR = target;
    enabled = 1;
}
void setTargetLspeed(int16_t target) {
    targetL = target;
    enabled = 1;
}

void setKp(int8_t coeff) {
    Kp = coeff;
    modifiedK |= MODIFIED_KP_MASK;
}
void setKi(int8_t coeff) {
    Ki = coeff;
    modifiedK |= MODIFIED_KI_MASK;
}
void setKd(int8_t coeff) {
    Kd = coeff;
    modifiedK |= MODIFIED_KD_MASK;
}
int8_t getKp() {
    return Kp;
}
int8_t getKi() {
    return Ki;
}
int8_t getKd() {
    return Kd;
}

void enablePID(uint8_t isEnabled) {
    enabled = isEnabled;
}


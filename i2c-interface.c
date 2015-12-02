#include "i2c-interface.h"
#include "e-i2c.h"
#include "motors.h"
#include "PIDcompute.h"
#include "mcc_generated_files/ccp4.h"
#include "mcc_generated_files/ccp5.h"
#include "mcc_generated_files/pin_manager.h"

void clearCoderCounters();
int8_t getStatus();

// initialize e-i2c and configure the handlers here
void configureI2Cinterface() {
    ei2cInit(); // initialize e-I2C library
    
    // simple write commands
    map0write(CLRCOD, clearCoderCounters);
    
    //8 bit write commands
    map8write(KP, setKp);
    map8write(KI, setKi);
    map8write(KD, setKd);
    
    //8 bit read commands
    map8read(DRIVER_STATUS, getStatus);
    map8read(KP, getKp);
    map8read(KI, getKi);
    map8read(KD, getKd);
    
    // 16 bit write commands
    map16write(MOTL, setMotorLi2c);
    map16write(MOTR, setMotorRi2c);
    map16write(SPEEDL, setTargetLspeed);
    map16write(SPEEDR, setTargetRspeed);
    map16write(CODL, setLticks);
    map16write(CODR, setRticks);
    
    //16 bit read commands
    map16read(SPEEDL, getLspeedINT);
    map16read(SPEEDR, getRspeedINT);
    map16read(CODL, getLticks);
    map16read(CODR, getRticks);
}

void clearCoderCounters() {
    setLticks(0);
    setRticks(0);
}

// return current status and clear INTOUT flag
// status REG : bit 0: isStalled
int8_t getStatus() {
    INTOUT_SetDigitalInput();
    if(stalled) {
        stalled = 0; // clear stalled INT flag
        return 1;
    } else
        return 0;
} 
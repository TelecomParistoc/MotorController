
#ifndef MOTORS_H
#define	MOTORS_H

#include <xc.h>
#include <stdint.h>

#define THRESHOLD 100
#ifdef	__cplusplus
extern "C" {
#endif

    void setMotorL(int16_t speed);
    void setMotorR(int16_t speed);
    
    // return a non zero value if left motor DC is greater than 127
    uint8_t motorLtryingToMove();
    
    // return a non zero value if right motor DC is greater than 127
    uint8_t motorRtryingToMove();
    
    // call whenever the robot is stopped : check it's not trying to move
    void stallDetectL();
    void stallDetectR();
    
    // directly set duty cycle from i2c command, disable PID
    void setMotorLi2c(int16_t speed);
    void setMotorRi2c(int16_t speed);
    
    uint8_t stalled;
    
#ifdef	__cplusplus
}
#endif

#endif	/* MOTORS_H */


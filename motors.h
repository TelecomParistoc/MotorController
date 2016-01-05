
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
       
#ifdef	__cplusplus
}
#endif

#endif	/* MOTORS_H */


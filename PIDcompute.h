/* 
 * File:   PIcompute.h
 * Author: Arnaud
 *
 * Created on October 22, 2015, 6:28 PM
 */

#ifndef PICOMPUTE_H
#define	PICOMPUTE_H

#include <xc.h>
#include <stdint.h>

#define MAX_INTEGRAL 30000
#define MIN_INTEGRAL -30000

#define EE_Kp_ADDR 0x01
#define EE_Ki_ADDR 0x02
#define EE_Kd_ADDR 0x03

#define MODIFIED_KD_MASK 0x01
#define MODIFIED_KI_MASK 0x02
#define MODIFIED_KP_MASK 0x04

#ifdef	__cplusplus
extern "C" {
#endif
    

    void PIDmanager(void);
    int16_t computePID(int16_t speed, int16_t targetSpeed, int16_t* integral, int16_t * lastError);
    
    void setTargetRspeed(int16_t target);
    void setTargetLspeed(int16_t target);
    
    void setKp(int8_t coeff);
    void setKi(int8_t coeff);
    void setKd(int8_t coeff);
    int8_t getKp();
    int8_t getKi();
    int8_t getKd();
    
    void enablePID(uint8_t);
    void PIDloadCoeffs(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* PICOMPUTE_H */


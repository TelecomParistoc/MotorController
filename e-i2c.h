
#ifndef E_I2C_H
#define	E_I2C_H

#include <xc.h>
#include <stdint.h>
#include <stddef.h>
#include "mcc_generated_files/i2c1.h"

#ifdef	__cplusplus
extern "C" {
#endif

    typedef enum {
        COMMAND = 0,
        FIRST_BYTE = 1,
        SECOND_BYTE = 2,
        REST = 3
    } msg_state;
    
    typedef enum {
        W16,
        W8,
        W0,
        R16,
        R8
    } msg_type;
    
    typedef union {
        struct {
            uint8_t lbyte;
            uint8_t hbyte;
        };

        struct {
            int16_t value;
        };
    } i2c_int16;
    
    typedef struct {
        i2c_int16 arg;
        uint8_t cmd;
        msg_state state;
        msg_type type;
    } i2c_msg;

    typedef void (*w16handler)(int16_t);
    typedef void (*w8handler)(int8_t);
    typedef void (*w0handler)();
    typedef int8_t (*r8handler)();
    typedef int16_t (*r16handler)();
    
    void ei2cInit();
    
    void i2cEventHandler(I2C1_SLAVE_DRIVER_STATUS i2c_state);
    
    uint8_t map0write(uint8_t cmd, w0handler fnct);
    uint8_t map8write(uint8_t cmd, w8handler fnct);
    uint8_t map16write(uint8_t cmd, w16handler fnct);
    uint8_t map8read(uint8_t cmd, r8handler fnct);
    uint8_t map16read(uint8_t cmd, r16handler fnct);
    
#ifdef	__cplusplus
}
#endif

#endif	/* E_I2C_H */


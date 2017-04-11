#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H
#include "ch.h"
#include "hal.h"

#define I2C_SLAVE_ADDRESS 0x12
#define I2C_THREAD_STACK_SIZE 1024

extern THD_WORKING_AREA(wa_i2c, I2C_THREAD_STACK_SIZE);
extern THD_FUNCTION(i2c_thread, i2cp);

#endif /* I2C_INTERFACE_H */

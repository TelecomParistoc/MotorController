#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H
#include "ch.h"
#include "hal.h"

#include "i2c_interface_addr.h"

#define I2C_SLAVE_ADDRESS 0x12

extern void i2c_slave_init (I2CDriver* i2cp);

#endif /* I2C_INTERFACE_H */

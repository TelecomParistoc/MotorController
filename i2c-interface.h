

#ifndef I2C_INTERFACE_H
#define	I2C_INTERFACE_H

/******************      SIMPLE COMMANDS      ******************/
//(no arguments) : have to be mapped between 0x00 and 0x3F
#define NUM_W0_CMD 1 // total number of commands, at least 1
// the higher command should be lower than 0x00 + NUM_W0_CMD

#define CLRCOD 0x01


/***************     8-bit GET/SET COMMANDS     ***************/
// (int8 arguments) : have to be mapped between 0x40 and 0x7F
#define NUM_W8_CMD 4 // total number of SET commands, at least 1
#define NUM_R8_CMD 4 // total number of GET commands, at least 1
// the higher command should be lower than 0x40 + NUM_x8_CMD

// GET : the reason for INTOUT : bit 0: robot is stalled
#define DRIVER_STATUS 0x40
// PID coefficients : GET/SET, KD is actually -Kd
#define KP 0x41
#define KI 0x42
#define KD 0x43


/***************     16-bit GET/SET COMMANDS     ***************/
// (int16 arguments) : have to be mapped between 0x80 and 0xFF
#define NUM_W16_CMD 7 // total number of SET commands, at least 1
#define NUM_R16_CMD 7 // total number of GET commands, at least 1
// the higher command should be lower than 0x80 + NUM_W16_CMD

// SET : PID speed control : argument = 172.34 * speed (speed in m/s), full signed int16
// GET : actual speed, returns value = = 172.34 * speed (speed in m/s), signed int
#define SPEEDL 0x83
#define SPEEDR 0x84
// CODER counter :  GET/SET coder counter, 128 ticks/rotation
// value = 1.52* distance (in mm)
#define CODL 0x85
#define CODR 0x86

// initialize e-i2c and configure the handlers here
void configureI2Cinterface();

#endif	/* I2C_INTERFACE_H */


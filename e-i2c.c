#include "e-i2c.h"
#include "mcc_generated_files/i2c1.h"
#include "i2c-interface.h"
#include <stdio.h>

void callHandler();

// handler pointer containers
w16handler w16cmds[NUM_W16_CMD];
w8handler w8cmds[NUM_W8_CMD];
w0handler w0cmds[NUM_W0_CMD];
r16handler r16cmds[NUM_R16_CMD];
r8handler r8cmds[NUM_R8_CMD];
// the most recent message
i2c_msg currentMsg;


void ei2cInit() {
    uint8_t i;
    // clear all the handler containers
    for(i=0; i<NUM_W16_CMD; i++) {
        w16cmds[i] = NULL;
    }
    for(i=0; i<NUM_W8_CMD; i++) {
        w8cmds[i] = NULL;
    }
    for(i=0; i<NUM_R16_CMD; i++) {
        r16cmds[i] = NULL;
    }
    for(i=0; i<NUM_R8_CMD; i++) {
        r8cmds[i] = NULL;
    }
}

void i2cEventHandler(I2C1_SLAVE_DRIVER_STATUS i2c_state) {
    switch (i2c_state) {
        case I2C1_SLAVE_WRITE_REQUEST:
            currentMsg.state = COMMAND;
            break;
        case I2C1_SLAVE_WRITE_COMPLETED:
            if(currentMsg.state == COMMAND) {  // the first byte is the command
                currentMsg.cmd = SSP1BUF;
                //printf("cmd: 0x%x\n", currentMsg.cmd);
                currentMsg.state = FIRST_BYTE;
                if(!(SSP1BUF&0xC0)) { // if it's a simple command (no arguments)
                    currentMsg.type = W0;
                    callHandler();
                    currentMsg.state = REST;
                }
            } else if(currentMsg.state == FIRST_BYTE) {
                if(currentMsg.cmd&0x80) { // if it's a W16
                    //printf("W16 1\n");
                    currentMsg.type = W16;
                    currentMsg.arg.lbyte = SSP1BUF;
                    currentMsg.state = SECOND_BYTE;
                } else { // W8
                    //printf("W8\n");
                    currentMsg.type = W8;
                    currentMsg.arg.lbyte = SSP1BUF;
                    callHandler();
                    currentMsg.state = REST;
                }
            } else if(currentMsg.state == SECOND_BYTE) { // receive W16 high byte
                //printf("W16 2\n");
                currentMsg.arg.hbyte = SSP1BUF;
                callHandler();
                currentMsg.state = REST;
            }
            break;
        case I2C1_SLAVE_READ_REQUEST:
            if(currentMsg.state == FIRST_BYTE) {
                if(currentMsg.cmd&0x80) { // if it's a R16
                    //printf("R16 1\n");
                    currentMsg.type = R16;
                    callHandler();
                    currentMsg.state = SECOND_BYTE;
                } else { // R8
                    //printf("R8\n");
                    currentMsg.type = R8;
                    callHandler();
                    currentMsg.state = REST;
                }
                SSP1BUF = currentMsg.arg.lbyte;
            } else if(currentMsg.state == SECOND_BYTE) { // send R16 high byte
                //printf("R16 2\n");
                SSP1BUF = currentMsg.arg.hbyte;
                currentMsg.state = REST;
            } else {
                //printf("ERROR\n");
                currentMsg.state = REST;
                SSP1BUF = 0xAA;
            }
            break;
        case I2C1_SLAVE_READ_COMPLETED:
            //printf("READ OK\n");
            currentMsg.state = REST;
        default:;
    } // end switch(i2c_bus_state)
}

void callHandler() {
    uint8_t cmd = currentMsg.cmd;
    switch(currentMsg.type) {
        case W16:
            cmd = cmd & 0x7F;
            if(cmd < NUM_W16_CMD && w16cmds[cmd] != NULL) { // if the command is valid
                w16cmds[cmd](currentMsg.arg.value);
            }
            break;
        case W8:
            cmd = cmd & 0x3F;
            if(cmd < NUM_W8_CMD && w8cmds[cmd] != NULL) { // if the command is valid
                w8cmds[cmd](currentMsg.arg.lbyte);
            }
            break;
        case W0:
            if(cmd < NUM_W0_CMD && w0cmds[cmd] != NULL) { // if the command is valid
                w0cmds[cmd]();
            }
            break;
        case R16:
            cmd = cmd & 0x7F;
            if(cmd < NUM_R16_CMD && r16cmds[cmd] != NULL) { // if the command is valid
                currentMsg.arg.value = r16cmds[cmd]();
            } else {
                currentMsg.arg.value = 0xAAAA; // send back an error code
            }
            break;
        case R8:
            cmd = cmd & 0x3F;
            if(cmd < NUM_R8_CMD && r8cmds[cmd] != NULL) { // if the command is valid
                currentMsg.arg.lbyte = r8cmds[cmd]();
            } else {
                currentMsg.arg.value = 0xAAAA; // send back an error code
            }
            break;
    }
}

uint8_t map16write(uint8_t cmd, w16handler fnct) {
    if(cmd > 0x7F && cmd < 0x80+NUM_W16_CMD) {
        w16cmds[cmd&0x7F] = fnct;
    } else {
        return 1;
    }
    return 0;
}
uint8_t map8write(uint8_t cmd, w8handler fnct) {
    if(cmd > 0x3F && cmd < 0x40+NUM_W8_CMD) {
        w8cmds[cmd&0x3F] = fnct;
    } else {
        return 1;
    }
    return 0;
}
uint8_t map0write(uint8_t cmd, w0handler fnct) {
    if(cmd < NUM_W0_CMD) {
        w0cmds[cmd] = fnct;
    } else {
        return 1;
    }
    return 0;
}
uint8_t map16read(uint8_t cmd, r16handler fnct) {
    if(cmd > 0x7F && cmd < 0x80+NUM_R16_CMD) {
        r16cmds[cmd&0x7F] = fnct;
    } else {
        return 1;
    }
    return 0;
}
uint8_t map8read(uint8_t cmd, r8handler fnct) {
    if(cmd > 0x3F && cmd < 0x40+NUM_R8_CMD) {
        r8cmds[cmd&0x3F] = fnct;
    } else {
        return 1;
    }
    return 0;
}
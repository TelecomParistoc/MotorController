
#include "mcc.h"
#include "i2c1.h"
#include "../e-i2c.h"

#define I2C1_SLAVE_ADDRESS 35
#define I2C1_SLAVE_MASK    127

/**
 Section: Global Variables
 */

volatile uint8_t I2C1_slaveWriteData = 0x55;

/**
 Section: Local Functions
 */

void I2C1_Initialize(void) {
    // initialize the hardware
    // BF RCinprocess_TXcomplete; UA dontupdate; SMP Sample At Middle; P stopbit_notdetected; S startbit_notdetected; R_nW write_noTX; CKE Idle to Active; D_nA lastbyte_address; 
    SSP1STAT = 0x00;
    // SSPEN enabled; WCOL no_collision; SSPOV no_overflow; CKP Idle:Low, Active:High; SSPM 7 Bit; 
    SSP1CON1 = 0x26;
    // ACKSTAT received; RCEN disabled; RSEN disabled; ACKEN disabled; SEN disabled; ACKDT acknowledge; GCEN enabled; PEN disabled; 
    SSP1CON2 = 0x80;
    // BOEN disabled; AHEN disabled; SBCDE disabled; SDAHT 100ns; DHEN disabled; ACKTIM ackseq; PCIE disabled; SCIE disabled; 
    SSP1CON3 = 0x00;
    // MSK0 127; 
    SSP1MSK = (I2C1_SLAVE_MASK << 1); // adjust UI mask for R/nW bit            
    // SSP1ADD 35; 
    SSP1ADD = (I2C1_SLAVE_ADDRESS << 1); // adjust UI address for R/nW bit

    // clear the slave interrupt flag
    PIR1bits.SSP1IF = 0;
    // enable the master interrupt
    PIE1bits.SSP1IE = 1;

}

void I2C1_ISR(void) {
    uint8_t i2c_data = 0x55;

    // NOTE: The slave driver will always acknowledge
    //       any address match.

    PIR1bits.SSP1IF = 0; // clear the slave interrupt flag
    i2c_data = SSP1BUF; // read SSPBUF to clear BF
    if (1 == SSP1STATbits.R_nW) {
        if ((1 == SSP1STATbits.D_nA) && (1 == SSP1CON2bits.ACKSTAT)) {
            // callback routine can perform any post-read processing
            i2cEventHandler(I2C1_SLAVE_READ_COMPLETED);
        } else {
            // callback routine should write data into SSPBUF
            i2cEventHandler(I2C1_SLAVE_READ_REQUEST);
        }
    } else if (0 == SSP1STATbits.D_nA) {
        // this is an I2C address

        if (0x00 == i2c_data) {
            // this is the General Call address
            i2cEventHandler(I2C1_SLAVE_GENERAL_CALL_REQUEST);
        } else {
            // callback routine should prepare to receive data from the master
            i2cEventHandler(I2C1_SLAVE_WRITE_REQUEST);
        }
    } else {
        I2C1_slaveWriteData = i2c_data;

        // callback routine should process I2C1_slaveWriteData from the master
        i2cEventHandler(I2C1_SLAVE_WRITE_COMPLETED);
    }

    SSP1CON1bits.CKP = 1; // release SCL

}
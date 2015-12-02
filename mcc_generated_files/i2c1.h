
#ifndef _I2C1_H
#define _I2C1_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <xc.h>

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

    /**
      I2C Slave Driver Status

      @Summary
        Defines the different status that the slave driver has
        detected over the i2c bus.

      @Description
        This defines the different status that the slave driver has
        detected over the i2c bus. The status is passed to the
        I2C1_StatusCallback() callback function that is implemented by
        the user of the slave driver as a parameter to inform the user
        that there was a change in the status of the driver due to
        transactions on the i2c bus. User of the slave driver can use these
        to manage the read or write buffers.

     */

    typedef enum {
        I2C1_SLAVE_WRITE_REQUEST,
        I2C1_SLAVE_READ_REQUEST,
        I2C1_SLAVE_WRITE_COMPLETED,
        I2C1_SLAVE_READ_COMPLETED,
        I2C1_SLAVE_GENERAL_CALL_REQUEST,
    } I2C1_SLAVE_DRIVER_STATUS;

    typedef enum {
        WRITE_REG,
        WRITE_DATA,
        SLAVE_GENERAL_CALL,
    } SLAVE_WRITE_DATA_TYPE;
    
    
#define I2C1_SLAVE_DEFAULT_ADDRESS          35

    
    /**
        @Summary
            Initializes and enables the i2c slave instance : 1

        @Description
            This routine initializes the i2c slave driver instance for : 1
            index, making it ready for clients to open and use it.

        @Preconditions
            None

        @Param
            None

        @Returns
            None

        @Example
            <code>
                // initialize the i2c slave driver
                I2C1_Initialize();

            </code>
     */

    void I2C1_Initialize(void);

    /**
       @Summary
            This function process the I2C interrupts generated by
            bus activity

        @Description
            This function calls a callback function with 1 of 4
            possible parameters.
                I2C1_SLAVE_WRITE_REQUEST
                I2C1_SLAVE_READ_REQUEST
                I2C1_SLAVE_WRITE_COMPLETED
                I2C1_SLAVE_READ_COMPLETED

            The callback function should contain application specific
            code to process I2C bus activity from the I2C master.
            A basic EEPROM emulator is provided as an example.
     */

    void I2C1_ISR(void);

    /**
       @Summary
            This variable contains the last data written to the I2C slave
     */

    extern volatile uint8_t I2C1_slaveWriteData;


#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif

#endif  // _I2C1_H

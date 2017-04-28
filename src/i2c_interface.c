#include "i2c_interface.h"
#include "i2c_lld.h"
#include "i2cslave.h"

#define I2C_BUFFER_SIZE 2
#define NO_DATA 0xFF
/*
 * Buffer to receive the message sent by the I2C master.
 */
static volatile uint8_t rx_buffer[5] = {NO_DATA};

static virtual_timer_t i2c_vt;
/*
 * Buffer to prepate the response to send to the I2C master.
 */
static volatile uint8_t tx_buffer[I2C_BUFFER_SIZE] = {42, 37};

/*
 * Indicates whether an I2C error occurred.
 */
static volatile bool error = FALSE;

/*
 * Configuration of the I2C driver.
 */
static I2CConfig i2c_slave_cfg = {
    0x20420F13, /* I2C clock = 100kHz, see table 141 of the reference manual */
    0, /* Nothing to do, the driver will set the PE bit */
    0, /* Nothing to do, all fields controlled by the driver */
    NULL/* Slave mode */
};

static I2CSlaveMsgCB i2c_error, i2c_reply, i2c_address_match;
/*
 * The i2c_request object is used when a "write" request is received. The specified
 * buffer is filled with incoming data and then the handler is called.
 * The i2c_response object is used when a "read" request is received. The content of
 * the specified buffer is sent and then the handler is called (after the transmission thus).
 *
 * In the implementation proposed here, it's the i2c_request handler that fills
 * the tx_buffer with the appropriate value. This I2C slave thus follows the
 * read-after-write scheme.
 * If a direct read request is sent by the master, the returned value will be the
 * one set in the tx_buffer last time (by the last "read-after-write" request).
 *
 * According to the number of bytes received, the request handler (the handler
 * in i2c_request, called to handle the "write" requests) will decide whether it's
 * a true "write" request and thus copy the recevied data in the appropriate
 * variable or the first part of a "read-after-write" request in which case it
 * will fill the tx_buffer with the requested value and then update the i2c_response
 * object and the I2C slave driver for these changes to take effect before the "read"
 * part of the request is received and tx_buffer sent.
 */

/*
 * Data structure used to handle incoming "write" requests from an I2C master.
 */
const I2CSlaveMsg i2c_request = {
    sizeof(rx_buffer),
    rx_buffer,
    i2c_address_match,
    NULL,
    i2c_error
};

/*
 * Data structure used to send a response to an I2C master.
 */
I2CSlaveMsg i2c_response = {
    I2C_BUFFER_SIZE,
    tx_buffer,
    NULL,
    i2c_reply,
    i2c_error
};

static void i2c_vt_cb(void* param)
{
    (void)param;
    /* Process the write message received */
    if (rx_buffer[0] != NO_DATA) {
        tx_buffer[1] = 0x10;
        rx_buffer[0] = NO_DATA;
        rx_buffer[1] = NO_DATA;
        rx_buffer[2] = NO_DATA;
    }
}

static void i2c_address_match(I2CDriver* i2cp)
{
    (void)i2cp;
    if (rx_buffer[0] != NO_DATA) {
        /* Start of the read part of a read-after-write exchange */
        chSysLockFromISR();
        chVTResetI(&i2c_vt);
        chSysUnlockFromISR();
        /* Prepare the answer */
        tx_buffer[0] = rx_buffer[0];

        /* Free the rx buffer */
        rx_buffer[0] = NO_DATA;
    } else {
        /* Start of a write exchange */
        chSysLockFromISR();
        chVTSetI(&i2c_vt, US2ST(300), i2c_vt_cb, NULL);
        chSysUnlockFromISR();
    }

}

static void i2c_reply(I2CDriver* i2cp) {
    (void)i2cp;
}

/*
 * @brief Handle an error in the I2C connection.
 */

void i2c_error(I2CDriver* i2cp)
{
    (void)i2cp;
    error = TRUE;
}

THD_WORKING_AREA(wa_i2c, I2C_THREAD_STACK_SIZE);
/*
 * The thread that acts as an I2C slave.
 */
extern THD_FUNCTION(i2c_thread, i2cp)
{
    if (i2cp == NULL) {
        return;
    } else {
        chVTObjectInit(&i2c_vt);
        i2cStart(i2cp, &i2c_slave_cfg);
        ((I2CDriver*)i2cp)->slaveTimeout = MS2ST(100);
        i2cSlaveConfigure(i2cp, &i2c_request, &i2c_response);
        i2cMatchAddress(i2cp, I2C_SLAVE_ADDRESS);

        while (error == FALSE)
        {
            chThdSleepMilliseconds(100);
    		palTogglePad(GPIOA, GPIOA_RUN_LED);
        }
    }
}

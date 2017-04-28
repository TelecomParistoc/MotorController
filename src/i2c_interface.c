#include "i2c_interface.h"
#include "i2c_lld.h"
#include "i2cslave.h"

#define I2C_TX_BUFFER_SIZE 2
#define I2C_RX_BUFFER_SIZE 5
#define NO_DATA 0xFF

/*
 * Buffer to receive the message sent by the I2C master.
 */
static volatile uint8_t rx_buffer[I2C_RX_BUFFER_SIZE];
/*
 * Buffer to prepate the response to send to the I2C master.
 */
static volatile uint8_t tx_buffer[I2C_TX_BUFFER_SIZE];

/*
 * Indicates whether an I2C error occurred.
 */
static volatile bool error = FALSE;

static virtual_timer_t i2c_vt;

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
 * The i2c_request object is used whenever a message is received. The
 * ```i2c_address_match``` handler is called just after the device address has
 * been recognized as one of the device addresses. This handler simply starts a
 * timer so that the callback associated with this timer is called after the end
 * of the message. It would have been much better to have a handler called at
 * the end of the message but this functionnality doesn't seem to work for the
 * time being. This solution is thus a dirty work-around.
 *
 * The i2c_response object is used only when a "read" request is received. The
 * content of the specified buffer is sent and then the ```i2c_reply``` handler
 * is called (after the transmission thus).
 *
 * In the implementation proposed here, it's the i2c_request handler that fills
 * the tx_buffer with the appropriate value. Indeed, this I2C slave follows the
 * read-after-write scheme. A "read" request is thus preceded by a "write" request,
 * with the address of the 'register' to read. When the read request is received,
 * the ```i2c_address_match``` handler is called and the rx_buffer contains the
 * address of the register to read. The handler can thus analyse this address and
 * fill the tx_buffer with the appropriate value before the content of this buffer
 * is sent to the I2C master. In that case, the handler also stops the timer that
 * has been started when the "write" request with the register address has been
 * received because it's in fact not a true "write" request.
 *
 * If a direct read request is sent by the master, the returned value will be the
 * one set in the tx_buffer last time (by the last "read-after-write" request).
 */

/*
 * Data structure used to handle incoming requests from an I2C master.
 */
const I2CSlaveMsg i2c_request = {
    I2C_RX_BUFFER_SIZE,
    rx_buffer,
    i2c_address_match,
    NULL,
    i2c_error
};

/*
 * Data structure used to send a response to an I2C master.
 */
I2CSlaveMsg i2c_response = {
    I2C_TX_BUFFER_SIZE,
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

/*
 * @brief Handler called when a "read" request has been served.
 */
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

/*
 * @brief Start the I2C driver.
 */
extern void i2c_slave_init(I2CDriver*  i2cp)
{
    int i;
    if (i2cp == NULL) {
        return;
    } else {
        /* Create the timer */
        chVTObjectInit(&i2c_vt);

        /* Initialise the buffers */
        tx_buffer[0] = 42;
        tx_buffer[1] = 37;

        for (i = 0; i < I2C_RX_BUFFER_SIZE; ++i) {
            rx_buffer[i] = NO_DATA;
        }

        /* Start the I2C driver */
        ((I2CDriver*)i2cp)->slaveTimeout = MS2ST(100);
        i2cStart(i2cp, &i2c_slave_cfg);
        i2cSlaveConfigure(i2cp, &i2c_request, &i2c_response);
        i2cMatchAddress(i2cp, I2C_SLAVE_ADDRESS);
    }
}

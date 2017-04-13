#include "i2c_interface.h"
#include "i2c_lld.h"
#include "i2cslave.h"

#define I2C_BUFFER_SIZE 100

/*
 * Buffer to receive the message sent by the I2C master.
 */
static uint8_t rx_buffer[I2C_BUFFER_SIZE];

/*
 * Buffer to prepate the response to send to the I2C master.
 */
static uint8_t tx_buffer[I2C_BUFFER_SIZE];

/*
 * Indicates whether an I2C error occurred.
 */
static volatile bool error = FALSE;

/*
 * Configuration of the I2C driver.
 */
static I2CConfig i2c_slave_cfg = {
    0x20420F13, /* to test */
    0x00000001, /* Peripheral enabled */
    0, /* Nothing to do, all fields controlled by the driver */
    NULL /* Slave mode */
};

static I2CSlaveMsgCB i2c_handler, i2c_error, i2c_reply;

/*
 * Data structure used to handle incoming request from an I2C master.
 */
const I2CSlaveMsg i2c_request = {
    I2C_BUFFER_SIZE,
    rx_buffer,
    NULL,
    i2c_handler,
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

/*
 * @brief Process an incoming request and prepare the response.
 *
 * @param[in] i2cp Pointer to the I2C driver.
 */
static void i2c_handler(I2CDriver* i2cp)
{
    /* Process the request which is stored in rx_buffer*/
    /* Write the response in tx_buffer */
    tx_buffer[0] = (uint8_t)'t';
    tx_buffer[1] = (uint8_t)'o';
    tx_buffer[2] = (uint8_t)'t';
    tx_buffer[3] = (uint8_t)'o';
    tx_buffer[4] = (uint8_t)'\0';
    i2c_response.size = 5;
    i2cSlaveReplyI(i2cp, &i2c_response);
}

/*
 * @brief Basic (void) function.
 */
void i2c_reply(I2CDriver* i2cp)
{
    (void)i2cp;
    i2c_response.size = 0;
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
        i2cStart(i2cp, &i2c_slave_cfg);
        ((I2CDriver*)i2cp)->slaveTimeout = MS2ST(100);
        i2cSlaveConfigure(i2cp, &i2c_request, &i2c_response);
        i2cMatchAddress(i2cp, I2C_SLAVE_ADDRESS);

        while (error == FALSE)
        {
            chThdSleepMilliseconds(100);
        }
    }
}

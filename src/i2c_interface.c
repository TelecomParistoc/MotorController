#include "i2c_interface.h"
#include "i2c_lld.h"
#include "i2cslave.h"
#include "settings.h"
#include "control.h"
#include "orientation.h"
#include "position.h"
#include "coding_wheels.h"
#include "data_storage.h"

#include <string.h>   //for memcpy

#include "log.h"

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
    (uint8_t*)rx_buffer,
    i2c_address_match,
    NULL,
    i2c_error
};

/*
 * Data structure used to send a response to an I2C master.
 */
I2CSlaveMsg i2c_response = {
    I2C_TX_BUFFER_SIZE,
    (uint8_t*)tx_buffer,
    NULL,
    i2c_reply,
    i2c_error
};

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

static void rx_special_cases(uint8_t addr) {
    static int32_t tmp_right_wheel_dist = 0;
    static int32_t tmp_left_wheel_dist = 0;
    static int32_t tmp_goal_mean_dist = 0;
    static int32_t tmp;

    LOG_VERBOSE("rx_buffer = 0x%x %d %d\n", rx_buffer[0], rx_buffer[1], rx_buffer[2]);
    /* Process the write message received */
    switch (addr) {
    case STORE_DATA_IN_FLASH_ADDR:
        store_data_in_flash();
        break;
    case RESET_ORIENTATION_ADDR:
        chSysLockFromISR();
        chBSemSignalI(&reset_orientation_sem);
        chSysUnlockFromISR();
        break;
    case CUR_RIGHT_WHEEL_DIST_LOW_ADDR:
        tmp_right_wheel_dist = (rx_buffer[2] << 8) | rx_buffer[1];
        break;
    case CUR_RIGHT_WHEEL_DIST_HIGH_ADDR:
        tmp_right_wheel_dist |= (rx_buffer[2] << 24) | (rx_buffer[1] << 16);
        right_ticks = tmp_right_wheel_dist * settings.ticks_per_m / 100;
        break;
    case CUR_LEFT_WHEEL_DIST_LOW_ADDR:
        tmp_left_wheel_dist = (rx_buffer[2] << 8) | rx_buffer[1];
        break;
    case CUR_LEFT_WHEEL_DIST_HIGH_ADDR:
        tmp_left_wheel_dist |= (rx_buffer[2] << 24) | (rx_buffer[1] << 16);
        left_ticks = tmp_left_wheel_dist * settings.ticks_per_m / 100;
        break;
    case CUR_HEADING_ADDR:
        tmp = ((rx_buffer[2] << 8) | rx_buffer[1]);
        if (tmp < 0 || tmp >= 360) {
          LOG_ERROR("Invalid argument for setHeading (must be in range [0; 360[): %d", tmp);
        }
        /* convert degrees into IMU unit */
        tmp *= 16;
        /* do not call set_orientation because this function calls getHeading
          and so uses the I2C bus which is already used by the control thread */
        orientation_changed = tmp;
        goal.heading = tmp;
        LOG_DEBUG("===============\n===============\n\nHeading set to %d\n\n", tmp);
        break;
    case GOAL_MEAN_DIST_LOW_ADDR:
        tmp_goal_mean_dist = (rx_buffer[2] << 8) | rx_buffer[1];
        break;
    case GOAL_MEAN_DIST_HIGH_ADDR:
        tmp_goal_mean_dist |= (rx_buffer[2] << 24) | (rx_buffer[1] << 16);
        goal.mean_dist = tmp_goal_mean_dist;
        dist_command_received = TRUE;
        break;
    case GOAL_HEADING_ADDR:
        /* Ignore if not valid */
        if (((rx_buffer[2] << 8) | rx_buffer[1]) <= 360) {
            /* IMU unit = 16 * degree */
            goal.heading = ((rx_buffer[2] << 8) | rx_buffer[1]) * 16;
        }
        else {
          LOG_ERROR("Invalid GOAL_HEADING received : rx_buffer = 0x%x; %d; %d\n",
                  rx_buffer[0], rx_buffer[1], rx_buffer[2]);
        }
        break;
    default:
        break;
    }
}

static void tx_special_cases(uint8_t addr, uint16_t *value) {
    static int32_t saved_left_wheel_dist = 0;
    static int32_t saved_right_wheel_dist = 0;

    switch (addr) {
    case CUR_RIGHT_WHEEL_DIST_LOW_ADDR:
        saved_right_wheel_dist = 100 * right_ticks / settings.ticks_per_m;
        *value = saved_right_wheel_dist & 0x0000FFFF;
        break;
    case CUR_RIGHT_WHEEL_DIST_HIGH_ADDR:
        *value = (saved_right_wheel_dist & 0xFFFF0000) >> 16U;
        break;
    case CUR_LEFT_WHEEL_DIST_LOW_ADDR:
        saved_left_wheel_dist = 100 * left_ticks / settings.ticks_per_m;
        *value = saved_left_wheel_dist & 0x0000FFFF;
        break;
    case CUR_LEFT_WHEEL_DIST_HIGH_ADDR:
        *value = (saved_left_wheel_dist & 0xFFFF0000) >> 16U;
        break;
    case CUR_HEADING_ADDR:
        *value = orientation / 16;
        break;
    case GOAL_HEADING_ADDR:
        *value = goal.heading / 16;
        break;
    default:
        *value = NO_DATA;
        break;
    }
}

/* Include the generated code */
#include "i2c_interface_gen.c"

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

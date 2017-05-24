#include "i2c_interface.h"
#include "i2c_lld.h"
#include "i2cslave.h"
#include "settings.h"
#include "control.h"
#include "orientation.h"
#include "position.h"
#include "coding_wheels.h"

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

static void i2c_vt_cb(void* param)
{
    (void)param;
    int32_t tmp_right_wheel_dist = 0;
    int32_t tmp_left_wheel_dist = 0;

    /* Process the write message received */
    if (rx_buffer[0] != NO_DATA) {
        switch(rx_buffer[0])
        {
        case WHEELS_GAP_ADDR:
            wheels_gap = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case TICKS_PER_M_ADDR:
            ticks_per_m = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_TRUST_THRESHOLD_ADDR:
            angular_trust_threshold = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case MAX_LINEAR_ACCELERATION_ADDR:
            max_linear_acceleration = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case MAX_ANGULAR_ACCELERATION_ADDR:
            max_angular_acceleration = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CRUISE_LINEAR_SPEED_ADDR:
            cruise_linear_speed = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CRUISE_ANGULAR_SPEED_ADDR:
            cruise_angular_speed = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case LINEAR_P_COEFF_ADDR:
            linear_p_coeff = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case LINEAR_I_COEFF_ADDR:
            linear_i_coeff = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case LINEAR_D_COEFF_ADDR:
            linear_d_coeff = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_P_COEFF_ADDR:
            angular_p_coeff = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_I_COEFF_ADDR:
            angular_i_coeff = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_D_COEFF_ADDR:
            angular_d_coeff = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CUR_RIGHT_WHEEL_DIST_LOW_ADDR:
            tmp_right_wheel_dist = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CUR_RIGHT_WHEEL_DIST_HIGH_ADDR:
            tmp_right_wheel_dist |= (rx_buffer[1] << 24) | (rx_buffer[2] << 16);
            right_ticks = tmp_right_wheel_dist * ticks_per_m / 100;
            break;
        case CUR_LEFT_WHEEL_DIST_LOW_ADDR:
            tmp_left_wheel_dist = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CUR_LEFT_WHEEL_DIST_HIGH_ADDR:
            tmp_left_wheel_dist |= (rx_buffer[1] << 24) | (rx_buffer[2] << 16);
            left_ticks = tmp_left_wheel_dist * ticks_per_m / 100;
            break;
        case CUR_HEADING_ADDR:
            orientation = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case GOAL_MEAN_DIST_ADDR:
            goal_mean_dist = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case GOAL_HEADING_ADDR:
            /* Ignore if not valid */
            if (((rx_buffer[2] << 8) | rx_buffer[1]) <= HEADING_MAX_VALUE) {
                goal_heading = (rx_buffer[2] << 8) | rx_buffer[1];
            }
            break;
        case HEADING_DIST_SYNC_REF_ADDR:
            heading_dist_sync_ref = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case MASTER_STOP_ADDR:
            master_stop = rx_buffer[1];
            break;
        default:
            break;
        }
    }

    /* Free the rx_buffer */
    rx_buffer[0] = NO_DATA;
    rx_buffer[1] = NO_DATA;
    rx_buffer[2] = NO_DATA;
}

static void i2c_address_match(I2CDriver* i2cp)
{
    (void)i2cp;
    uint16_t value;

    /*
     * 32 bits value are accessed in 2 messages. The value used must be the
     * same so that it remains coherent.
     */
    uint32_t saved_cur_x = 0U;
    uint32_t saved_cur_y = 0U;
    int32_t saved_left_wheel_dist = 0;
    int32_t saved_right_wheel_dist = 0;
    int32_t saved_cur_dist = 0;
    bool single_byte = FALSE;

    if (rx_buffer[0] != NO_DATA) {
        /* Start of the read part of a read-after-write exchange */
        chSysLockFromISR();
        chVTResetI(&i2c_vt);
        chSysUnlockFromISR();

        /* Prepare the answer */
        switch (rx_buffer[0]) {
        case WHEELS_GAP_ADDR:
            value = wheels_gap;
            break;
        case TICKS_PER_M_ADDR:
            value = ticks_per_m;
            break;
        case ANGULAR_TRUST_THRESHOLD_ADDR:
            value = angular_trust_threshold;
            break;
        case MAX_LINEAR_ACCELERATION_ADDR:
            value = max_linear_acceleration;
            break;
        case MAX_ANGULAR_ACCELERATION_ADDR:
            value = max_angular_acceleration;
            break;
        case CRUISE_LINEAR_SPEED_ADDR:
            value = cruise_linear_speed;
            break;
        case CRUISE_ANGULAR_SPEED_ADDR:
            value = cruise_angular_speed;
            break;
        case LINEAR_P_COEFF_ADDR:
            value = linear_p_coeff;
            break;
        case LINEAR_I_COEFF_ADDR:
            value = linear_i_coeff;
            break;
        case LINEAR_D_COEFF_ADDR:
            value = linear_d_coeff;
            break;
        case ANGULAR_P_COEFF_ADDR:
            value = angular_p_coeff;
            break;
        case ANGULAR_I_COEFF_ADDR:
            value = angular_i_coeff;
            break;
        case ANGULAR_D_COEFF_ADDR:
            value = angular_d_coeff;
            break;
        case CUR_ABS_X_LOW_ADDR:
            saved_cur_x = current_x;
            value = saved_cur_x & 0x0000FFFFU;
            break;
        case CUR_ABS_X_HIGH_ADDR:
            value = (saved_cur_x & 0xFFFF0000U) >> 16;
            break;
        case CUR_ABS_Y_LOW_ADDR:
            saved_cur_y = current_y;
            value = saved_cur_y & 0x0000FFFFU;
            break;
        case CUR_ABS_Y_HIGH_ADDR:
            value = (saved_cur_y & 0xFFFF0000) >> 16;
            break;
        case CUR_RIGHT_WHEEL_DIST_LOW_ADDR:
            saved_right_wheel_dist = 100 * right_ticks / ticks_per_m;
            value = saved_right_wheel_dist & 0x0000FFFF;
            break;
        case CUR_RIGHT_WHEEL_DIST_HIGH_ADDR:
            value = (saved_right_wheel_dist & 0xFFFF0000) >> 16;
            break;
        case CUR_LEFT_WHEEL_DIST_LOW_ADDR:
            saved_left_wheel_dist = 100 * left_ticks / ticks_per_m;
            value = saved_left_wheel_dist & 0x0000FFFF;
            break;
        case CUR_LEFT_WHEEL_DIST_HIGH_ADDR:
            value = (saved_left_wheel_dist & 0xFFFF0000) >> 16;
            break;
        case CUR_HEADING_ADDR:
            value = orientation;
            break;
        case CUR_DIST_LOW_ADDR:
            saved_cur_dist = current_distance;
            value = (saved_cur_dist & 0x0000FFFF);
            break;
        case CUR_DIST_HIGH_ADDR:
            value = (saved_cur_dist & 0xFFFF0000) >> 16;
            break;
        /* The next 3 should be write-only according to specs */
        case GOAL_MEAN_DIST_ADDR:
            value = goal_mean_dist;
            break;
        case GOAL_HEADING_ADDR:
            value = goal_heading;
            break;
        case HEADING_DIST_SYNC_REF_ADDR:
            value = heading_dist_sync_ref;
            break;
        case MASTER_STOP_ADDR:
            single_byte = TRUE;
            tx_buffer[0] = master_stop;
            break;
        default:
            single_byte = TRUE;
            value = NO_DATA;
            break;
        }

        if (single_byte == FALSE) {
            tx_buffer[0] = (uint8_t)((value & 0xFF00) >> 8);
            tx_buffer[1] = (uint8_t)(value & 0x00FF);
            i2c_response.size = 2U;
        } else {
            i2c_response.size = 1U;
        }

        /* Free the rx buffer */
        rx_buffer[0] = NO_DATA;
    } else {
        /* Start of a write exchange */
        chSysLockFromISR();
        chVTSetI(&i2c_vt, US2ST(800), i2c_vt_cb, NULL);
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

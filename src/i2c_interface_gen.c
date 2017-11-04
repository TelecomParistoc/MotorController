#ifndef I2C_INTERFACE_GEN_C
#define I2C_INTERFACE_GEN_C

static void i2c_vt_cb(void* param)
{
    (void)param;
    static int32_t tmp_cur_x = 0;
    static int32_t tmp_cur_y = 0;
    static int32_t tmp_coding_wheel_left_initial_ticks = 0;
    static int32_t tmp_coding_wheel_right_initial_ticks = 0;

    /* Process the write message received */
    if (rx_buffer[0] != NO_DATA) {
        switch(rx_buffer[0])
        {
        case WHEELS_GAP_ADDR:
            settings.wheels_gap = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case TICKS_PER_M_ADDR:
            settings.ticks_per_m = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_TRUST_THRESHOLD_ADDR:
            settings.angular_trust_threshold = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case MAX_LINEAR_ACCELERATION_ADDR:
            settings.max_linear_acceleration = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case MAX_ANGULAR_ACCELERATION_ADDR:
            settings.max_angular_acceleration = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CRUISE_LINEAR_SPEED_ADDR:
            settings.cruise_linear_speed = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CRUISE_ANGULAR_SPEED_ADDR:
            settings.cruise_angular_speed = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case LINEAR_P_COEFF_ADDR:
            settings.linear_coeff.p = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case LINEAR_I_COEFF_ADDR:
            settings.linear_coeff.i = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case LINEAR_D_COEFF_ADDR:
            settings.linear_coeff.d = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_P_COEFF_ADDR:
            settings.angular_coeff.p = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_I_COEFF_ADDR:
            settings.angular_coeff.i = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case ANGULAR_D_COEFF_ADDR:
            settings.angular_coeff.d = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case MOTOR_LEFT_FORWARD_SENSE_ADDR:
            settings.motor_left_forward_sense = rx_buffer[1];
            break;
        case MOTOR_RIGHT_FORWARD_SENSE_ADDR:
            settings.motor_right_forward_sense = rx_buffer[1];
            break;
        case CODING_WHEEL_LEFT_INITIAL_TICKS_ADDR_LOW:
            tmp_coding_wheel_left_initial_ticks = (rx_buffer[2] << 8U) | rx_buffer[1];
            break;
        case CODING_WHEEL_LEFT_INITIAL_TICKS_ADDR_HIGH:
            tmp_coding_wheel_left_initial_ticks |= ((rx_buffer[2] << 24U) | (rx_buffer[1] << 16U));
            settings.coding_wheels_config.initial_left_ticks = tmp_coding_wheel_left_initial_ticks;
            break;
        case CODING_WHEEL_RIGHT_INITIAL_TICKS_ADDR_LOW:
            tmp_coding_wheel_right_initial_ticks = (rx_buffer[2] << 8U) | rx_buffer[1];
            break;
        case CODING_WHEEL_RIGHT_INITIAL_TICKS_ADDR_HIGH:
            tmp_coding_wheel_right_initial_ticks |= ((rx_buffer[2] << 24U) | (rx_buffer[1] << 16U));
            settings.coding_wheels_config.initial_right_ticks = tmp_coding_wheel_right_initial_ticks;
            break;
        case CODING_WHEEL_LEFT_ORIENTATION_ADDR:
            settings.coding_wheels_config.left_wheel_orientation = rx_buffer[1];
            break;
        case CODING_WHEEL_RIGHT_ORIENTATION_ADDR:
            settings.coding_wheels_config.right_wheel_orientation = rx_buffer[1];
            break;
        case CUR_ABS_X_LOW_ADDR:
            tmp_cur_x = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CUR_ABS_X_HIGH_ADDR:
            tmp_cur_x |= (rx_buffer[2] << 24) | (rx_buffer[1] << 16);
            cur_pos.x = tmp_cur_x;
            break;
        case CUR_ABS_Y_LOW_ADDR:
            tmp_cur_y = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case CUR_ABS_Y_HIGH_ADDR:
            tmp_cur_y |= (rx_buffer[2] << 24) | (rx_buffer[1] << 16);
            cur_pos.y = tmp_cur_y;
            break;
        case HEADING_DIST_SYNC_REF_ADDR:
            goal.heading_dist_sync_ref = (rx_buffer[2] << 8) | rx_buffer[1];
            break;
        case MASTER_STOP_ADDR:
            master_stop = rx_buffer[1];
            break;
        default:
            rx_special_cases(rx_buffer[0]);
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
    static uint32_t saved_cur_x = 0U;
    static uint32_t saved_cur_y = 0U;
    static int32_t saved_cur_dist = 0;
    static int32_t saved_goal_mean_dist = 0;
    static int32_t saved_coding_wheel_left_initial_ticks = 0;
    static int32_t saved_coding_wheel_right_initial_ticks = 0;
    bool single_byte = FALSE;

    if (rx_buffer[0] != NO_DATA) {
        /* Start of the read part of a read-after-write exchange */
        chSysLockFromISR();
        chVTResetI(&i2c_vt);
        chSysUnlockFromISR();

        /* Prepare the answer */
        switch (rx_buffer[0]) {
        case WHEELS_GAP_ADDR:
            value = settings.wheels_gap;
            break;
        case TICKS_PER_M_ADDR:
            value = settings.ticks_per_m;
            break;
        case ANGULAR_TRUST_THRESHOLD_ADDR:
            value = settings.angular_trust_threshold;
            break;
        case MAX_LINEAR_ACCELERATION_ADDR:
            value = settings.max_linear_acceleration;
            break;
        case MAX_ANGULAR_ACCELERATION_ADDR:
            value = settings.max_angular_acceleration;
            break;
        case CRUISE_LINEAR_SPEED_ADDR:
            value = settings.cruise_linear_speed;
            break;
        case CRUISE_ANGULAR_SPEED_ADDR:
            value = settings.cruise_angular_speed;
            break;
        case LINEAR_P_COEFF_ADDR:
            value = settings.linear_coeff.p;
            break;
        case LINEAR_I_COEFF_ADDR:
            value = settings.linear_coeff.i;
            break;
        case LINEAR_D_COEFF_ADDR:
            value = settings.linear_coeff.d;
            break;
        case ANGULAR_P_COEFF_ADDR:
            value = settings.angular_coeff.p;
            break;
        case ANGULAR_I_COEFF_ADDR:
            value = settings.angular_coeff.i;
            break;
        case ANGULAR_D_COEFF_ADDR:
            value = settings.angular_coeff.d;
            break;
        case MOTOR_LEFT_FORWARD_SENSE_ADDR:
            value = settings.motor_left_forward_sense;
            single_byte = TRUE;
            break;
        case MOTOR_RIGHT_FORWARD_SENSE_ADDR:
            value = settings.motor_right_forward_sense;
            single_byte = TRUE;
            break;
        case CODING_WHEEL_LEFT_INITIAL_TICKS_ADDR_LOW:
            saved_coding_wheel_left_initial_ticks = settings.coding_wheels_config.initial_left_ticks;
            value = saved_coding_wheel_left_initial_ticks & 0x0000FFFFU;
            break;
        case CODING_WHEEL_LEFT_INITIAL_TICKS_ADDR_HIGH:
            value = (saved_coding_wheel_left_initial_ticks & 0xFFFF0000U) >> 16U;
            break;
        case CODING_WHEEL_RIGHT_INITIAL_TICKS_ADDR_LOW:
            saved_coding_wheel_right_initial_ticks = settings.coding_wheels_config.initial_right_ticks;
            value = saved_coding_wheel_right_initial_ticks & 0x0000FFFFU;
            break;
        case CODING_WHEEL_RIGHT_INITIAL_TICKS_ADDR_HIGH:
            value = (saved_coding_wheel_right_initial_ticks & 0xFFFF0000U) >> 16U;
            break;
        case CODING_WHEEL_LEFT_ORIENTATION_ADDR:
            value = settings.coding_wheels_config.left_wheel_orientation;
            single_byte = TRUE;
            break;
        case CODING_WHEEL_RIGHT_ORIENTATION_ADDR:
            value = settings.coding_wheels_config.right_wheel_orientation;
            single_byte = TRUE;
            break;
        case CUR_ABS_X_LOW_ADDR:
            saved_cur_x = cur_pos.x / 100;
            value = saved_cur_x & 0x0000FFFFU;
            break;
        case CUR_ABS_X_HIGH_ADDR:
            value = (saved_cur_x & 0xFFFF0000U) >> 16U;
            break;
        case CUR_ABS_Y_LOW_ADDR:
            saved_cur_y = cur_pos.y / 100;
            value = saved_cur_y & 0x0000FFFFU;
            break;
        case CUR_ABS_Y_HIGH_ADDR:
            value = (saved_cur_y & 0xFFFF0000) >> 16U;
            break;
        case CUR_DIST_LOW_ADDR:
            saved_cur_dist = current_distance;
            value = (saved_cur_dist & 0x0000FFFF);
            break;
        case CUR_DIST_HIGH_ADDR:
            value = (saved_cur_dist & 0xFFFF0000) >> 16U;
            break;
        /* The next 4 should be write-only according to specs */
        case GOAL_MEAN_DIST_LOW_ADDR:
            saved_goal_mean_dist = goal.mean_dist;
            value = (saved_goal_mean_dist & 0x0000FFFF);
            break;
        case GOAL_MEAN_DIST_HIGH_ADDR:
            value = (saved_goal_mean_dist & 0xFFFF0000) >> 16U;
            break;
        case HEADING_DIST_SYNC_REF_ADDR:
            value = goal.heading_dist_sync_ref;
            break;
        case MASTER_STOP_ADDR:
            value = master_stop;
            single_byte = TRUE;
            break;
        default:
            tx_special_cases(rx_buffer[0], &value);
            break;
        }

        if (single_byte == FALSE) {
            tx_buffer[1] = (uint8_t)((value & 0xFF00) >> 8);
            tx_buffer[0] = (uint8_t)(value & 0x00FF);
            i2c_response.size = 2U;
        } else {
            tx_buffer[0] = (uint8_t)(value & 0x00FF);
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

#endif /* I2C_INTERFACE_GEN_C */

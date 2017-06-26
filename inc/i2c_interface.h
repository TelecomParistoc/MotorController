#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H
#include "ch.h"
#include "hal.h"

#define WHEELS_GAP_ADDR                   0x00U
#define TICKS_PER_M_ADDR                  0x02U
#define ANGULAR_TRUST_THRESHOLD_ADDR      0x04U
#define MAX_LINEAR_ACCELERATION_ADDR      0x06U
#define MAX_ANGULAR_ACCELERATION_ADDR     0x08U
#define CRUISE_LINEAR_SPEED_ADDR          0x0AU
#define CRUISE_ANGULAR_SPEED_ADDR         0x0CU
#define LINEAR_P_COEFF_ADDR               0x0EU
#define LINEAR_I_COEFF_ADDR               0x10U
#define LINEAR_D_COEFF_ADDR               0x12U
#define ANGULAR_P_COEFF_ADDR              0x14U
#define ANGULAR_I_COEFF_ADDR              0x16U
#define ANGULAR_D_COEFF_ADDR              0x18U
#define STORE_DATA_IN_FLASH_ADDR          0x20U
#define CUR_ABS_X_LOW_ADDR                0x80U
#define CUR_ABS_X_HIGH_ADDR               0x82U
#define CUR_ABS_Y_LOW_ADDR                0x84U
#define CUR_ABS_Y_HIGH_ADDR               0x86U
#define CUR_RIGHT_WHEEL_DIST_LOW_ADDR     0x88U
#define CUR_RIGHT_WHEEL_DIST_HIGH_ADDR    0x8AU
#define CUR_LEFT_WHEEL_DIST_LOW_ADDR      0x8CU
#define CUR_LEFT_WHEEL_DIST_HIGH_ADDR     0x8EU
#define CUR_HEADING_ADDR                  0x90U
#define CUR_DIST_LOW_ADDR                 0x92U
#define CUR_DIST_HIGH_ADDR                0x94U
#define GOAL_MEAN_DIST_LOW_ADDR           0xA0U
#define GOAL_MEAN_DIST_HIGH_ADDR          0xA2U
#define GOAL_HEADING_ADDR                 0xA4U
#define HEADING_DIST_SYNC_REF_ADDR        0xA6U
#define MASTER_STOP_ADDR                  0xA8U

#define I2C_SLAVE_ADDRESS 0x12

extern void i2c_slave_init (I2CDriver* i2cp);

#endif /* I2C_INTERFACE_H */

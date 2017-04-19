#ifndef BOARD_H
#define BOARD_H

/*
 * Setup for Motorboardv2
 */

/*
 * Board identifier.
 */
#define BOARD_ST_STM32F3_DISCOVERY
#define BOARD_NAME                  "Motorboardv2"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                0U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#define STM32_LSE_BYPASS
#define STM32_HSE_BYPASS

/*
 * MCU type as defined in the ST header.
 */
#define STM32F302x8

/*
 * IO pins assignments.
 */
#define GPIOA_RUN_LED               0U
#define GPIOA_PIN1                  1U
#define GPIOA_RMOTPWM               2U
#define GPIOA_RMOTA                 3U
#define GPIOA_RMOTB                 4U
#define GPIOA_LMOTB                 5U
#define GPIOA_LMOTA                 6U
#define GPIOA_LMOTPWM               7U
#define GPIOA_PIN8                  8U
#define GPIOA_PIN9                  9U
#define GPIOA_PIN10                 10U
#define GPIOA_PIN11                 11U
#define GPIOA_PIN12                 12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_RAD_N_CS              15U

#define GPIOB_PIN0                  0U
#define GPIOB_PIN1                  1U
#define GPIOB_PIN2                  2U
#define GPIOB_RAD_SCK               3U
#define GPIOB_RAD_MISO              4U
#define GPIOB_RAD_MOSI              5U
#define GPIOB_PIN6                  6U
#define GPIOB_RPI_SDA               7U
#define GPIOB_RPI_SCL               8U
#define GPIOB_PIN9                  9U
#define GPIOB_RCODA                 10U
#define GPIOB_LCODA                 11U
#define GPIOB_RCODB                 12U
#define GPIOB_LCODB                 13U
#define GPIOB_PIN14                 14U
#define GPIOB_PIN15                 15U

#define GPIOF_IMU_SDA               0U
#define GPIOF_IMU_SCL               1U

/*
 * IO lines assignments.
 */
#define LINE_RUN_LED               PAL_LINE(GPIOA, GPIOA_RUN_LED)
#define LINE_RMOTA                 PAL_LINE(GPIOA, GPIOA_RMOTA)
#define LINE_RMOTB                 PAL_LINE(GPIOA, GPIOA_RMOTB)
#define LINE_LMOTB                 PAL_LINE(GPIOA, GPIOA_LMOTB)
#define LINE_LMOTA                 PAL_LINE(GPIOA, GPIOA_LMOTA)
#define LINE_RAD_N_CS              PAL_LINE(GPIOA, GPIOA_RAD_N_CS)

#define LINE_RCODA                 PAL_LINE(GPIOB, GPIOB_RCODA)
#define LINE_LCODA                 PAL_LINE(GPIOB, GPIOB_LCODA)
#define LINE_RCODB                 PAL_LINE(GPIOB, GPIOB_RCODB)
#define LINE_LCODB                 PAL_LINE(GPIOB, GPIOB_LCODB)

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

// [MODE OTYPE OSPEED PUPDR]

/*
 * GPIOA setup:
 *
 * PA0  - RUN_LED                   (output push-pull).
 * PA1  - PIN1                      (input floating).
 * PA2  - RMOTPWM                   (alternate 1).
 * PA3  - RMOTA                     (output push-pull).
 * PA4  - RMOTB                     (output push-pull).
 * PA5  - LMOTB                     (output push-pull).
 * PA6  - LMOTA                     (output push-pull).
 * PA7  - LMOTPWM                   (alternate 1).
 * PA8  - PIN8                      (input floating).
 * PA9  - PIN9                      (input floating).
 * PA10 - PIN10                     (input floating).
 * PA11 - PIN11                     (input floating).
 * PA12 - PIN12                     (input floating).
 * PA13 - SWDIO                     (alternate 0 pull-up).
 * PA14 - SWCLK                     (alternate 0 pull-down).
 * PA15 - RAD_N_CS                  (output push-pull).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_OUTPUT(GPIOA_RUN_LED) |       \
                                     PIN_MODE_INPUT(GPIOA_PIN1) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_RMOTPWM) |    \
                                     PIN_MODE_OUTPUT(GPIOA_RMOTA)|          \
                                     PIN_MODE_OUTPUT(GPIOA_RMOTB) |         \
                                     PIN_MODE_OUTPUT(GPIOA_LMOTB) |         \
                                     PIN_MODE_OUTPUT(GPIOA_LMOTA) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_LMOTPWM) |    \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOA_PIN12) |          \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_RAD_N_CS))
#define VAL_GPIOA_OTYPER    0x00000000
#define VAL_GPIOA_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_SWDIO)  |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK))
#define VAL_GPIOA_ODR       0x00000000
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_RMOTPWM, 1U) |       \
                                     PIN_AFIO_AF(GPIOA_LMOTPWM, 1U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SWDIO, 0U)   |       \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - PIN0                      (input floating).
 * PB1  - PIN1                      (input floating).
 * PB2  - PIN2                      (input floating).
 * PB3  - RAD_SCK                   (alternate 6).
 * PB4  - RAD_MISO                  (alternate 6).
 * PB5  - RAD_MOSI                  (alternate 6).
 * PB6  - PIN6                      (input floating).
 * PB7  - RPI_SDA                   (alternate 4 open drain).
 * PB8  - RPI_SCL                   (alternate 4 open drain).
 * PB9  - PIN9                      (input floating).
 * PB10 - RCODA                     (input pull-up).
 * PB11 - LCODA                     (input pull-up).
 * PB12 - RCODB                     (input pull-up).
 * PB13 - LCODB                     (input pull-up).
 * PB14 - PIN14                     (input floating).
 * PB15 - PIN15                     (input floating).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(GPIOB_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN2) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_RAD_SCK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_RAD_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_RAD_MOSI) |   \
                                     PIN_MODE_INPUT(GPIOB_PIN6) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_RPI_SDA) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_RPI_SCL) |    \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOB_RCODA) |          \
                                     PIN_MODE_INPUT(GPIOB_LCODA) |          \
                                     PIN_MODE_INPUT(GPIOB_RCODB) |          \
                                     PIN_MODE_INPUT(GPIOB_LCODB) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOB_PIN15))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_RPI_SDA) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_RPI_SCL))
#define VAL_GPIOB_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_RCODA) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_LCODA) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_RCODB) |        \
                                     PIN_PUPDR_PULLUP(GPIOB_LCODB))
#define VAL_GPIOB_ODR       0x00000000
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_RAD_SCK, 6U) |       \
                                     PIN_AFIO_AF(GPIOB_RAD_MISO, 6U) |      \
                                     PIN_AFIO_AF(GPIOB_RAD_MOSI, 6U) |      \
                                     PIN_AFIO_AF(GPIOB_RPI_SDA, 4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_RPI_SCL, 4U))

/*
 * GPIOC setup:
 *
 * PC13 - PIN13                     (input floating).
 * PC14 - PIN14                     (input floating).
 * PC15 - PIN15                     (input floating).
 */
#define VAL_GPIOC_MODER     0x00000000
#define VAL_GPIOC_OTYPER    0x00000000
#define VAL_GPIOC_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOC_PUPDR     0x00000000
#define VAL_GPIOC_ODR       0x00000000
#define VAL_GPIOC_AFRL      0x00000000
#define VAL_GPIOC_AFRH      0x00000000

/*
 * GPIOD and E doesn't exist but seems to be required by ChibiOS
 */
#define VAL_GPIOD_MODER     0x00000000
#define VAL_GPIOD_OTYPER    0x00000000
#define VAL_GPIOD_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOD_PUPDR     0x00000000
#define VAL_GPIOD_ODR       0x00000000
#define VAL_GPIOD_AFRL      0x00000000
#define VAL_GPIOD_AFRH      0x00000000
#define VAL_GPIOE_MODER     0x00000000
#define VAL_GPIOE_OTYPER    0x00000000
#define VAL_GPIOE_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOE_PUPDR     0x00000000
#define VAL_GPIOE_ODR       0x00000000
#define VAL_GPIOE_AFRL      0x00000000
#define VAL_GPIOE_AFRH      0x00000000

/*
 * GPIOF setup:
 *
 * PF0  - IMU_SDA                    (alternate 4 open drain).
 * PF1  - IMU_SCL                    (alternate 4 open drain).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_IMU_SDA) |    \
                                     PIN_MODE_ALTERNATE(GPIOF_IMU_SCL))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOF_IMU_SDA) |   \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_IMU_SCL))
#define VAL_GPIOF_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOF_PUPDR     0x00000000
#define VAL_GPIOF_ODR       0x00000000
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_IMU_SDA, 4U)  |      \
                                     PIN_AFIO_AF(GPIOF_IMU_SCL, 4U))
#define VAL_GPIOF_AFRH      0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */

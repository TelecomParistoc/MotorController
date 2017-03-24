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
#define GPIOA_VBAT_PROBE            0U
#define GPIOA_MTR_LED_L             1U
#define GPIOA_MTR_PHASE_L           2U
#define GPIOA_MTR_ENABLE_L          3U
#define GPIOA_PIN4                  4U
#define GPIOA_MTR_PHASE_R           5U
#define GPIOA_MTR_ENABLE_R          6U
#define GPIOA_MTR_TRA_L             7U
#define GPIOA_PIN8                  8U
#define GPIOA_USB_VBUS              9U
#define GPIOA_USB_CONNECT           10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_DWM_SPI_CSn           15U

#define GPIOB_MTR_TRA_R             0U
#define GPIOB_LED_IR_C              1U
#define GPIOB_PIN2                  2U
#define GPIOB_DWM_WAKEUP            3U
#define GPIOB_PIN4                  4U
#define GPIOB_PIN5                  5U
#define GPIOB_PIN6                  6U
#define GPIOB_MPU_INT               7U
#define GPIOB_MPU_I2C_SCL           8U
#define GPIOB_MPU_I2C_SDA           9U
#define GPIOB_LED_IR_R              10U
#define GPIOB_LED_IR_L              11U
#define GPIOB_PIN12                 12U
#define GPIOB_LED_SPI_CK            13U
#define GPIOB_PIN14                 14U
#define GPIOB_LED_SPI_DO            15U

#define GPIOC_PIN13                 13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOF_OSC_IN                0U
#define GPIOF_OSC_OUT               1U

/*
 * IO lines assignments.
 */
#define LINE_VBAT_PROBE            PAL_LINE(GPIOA, 0U)
#define LINE_MTR_LED_L             PAL_LINE(GPIOA, 1U)
#define LINE_MTR_PHASE_L           PAL_LINE(GPIOA, 2U)
#define LINE_MTR_ENABLE_L          PAL_LINE(GPIOA, 3U)
#define LINE_MTR_PHASE_R           PAL_LINE(GPIOA, 5U)
#define LINE_MTR_ENABLE_R          PAL_LINE(GPIOA, 6U)
#define LINE_MTR_TRA_L             PAL_LINE(GPIOA, 7U)
#define LINE_USB_VBUS              PAL_LINE(GPIOA, 9U)
#define LINE_USB_CONNECT           PAL_LINE(GPIOA, 10U)
#define LINE_USB_DM                PAL_LINE(GPIOA, 11U)
#define LINE_USB_DP                PAL_LINE(GPIOA, 12U)
#define LINE_DWM_CSn               PAL_LINE(GPIOA, 15U)

#define LINE_MTR_TRA_R             PAL_LINE(GPIOB, 0U)
#define LINE_LED_IR_C              PAL_LINE(GPIOB, 1U)
#define LINE_DWM_WAKEUP            PAL_LINE(GPIOB, 3U)
#define LINE_MPU_INT               PAL_LINE(GPIOB, 7U)
#define LINE_MPU_I2C_SCL           PAL_LINE(GPIOB, 8U)
#define LINE_MPU_I2C_SDA           PAL_LINE(GPIOB, 9U)
#define LINE_LED_IR_R              PAL_LINE(GPIOB, 10U)
#define LINE_LED_IR_L              PAL_LINE(GPIOB, 11U)
#define LINE_LED_CK                PAL_LINE(GPIOB, 13U)
#define LINE_LED_DO                PAL_LINE(GPIOB, 15U)


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
 * PA0  - VBAT_PROBE                (analog).
 * PA1  - MTR_LED_L                 (output push-pull).
 * PA2  - MTR_PHASE_L               (alternate 9, pull-down).
 * PA3  - MTR_ENABLE_L              (alternate 9, pull-down).
 * PA4  - PIN4                      (input floating).
 * PA5  - MTR_PHASE_R               (alternate 1, pull-down).
 * PA6  - MTR_ENABLE_R              (alternate 1, pull-down).
 * PA7  - MTR_TRA_L                 (analog) comparator ADC
 * PA8  - PIN8                      (input floating).
 * PA9  - USB_VBUS                  (input floating) auto set by usb init
 * PA10 - USB_CONNECT               (output push-pull).
 * PA11 - USB_DM                    (input floating) auto set by usb init
 * PA12 - USB_DP                    (input floating) auto set by usb init
 * PA13 - SWDIO                     (alternate 0 pull-up).
 * PA14 - SWCLK                     (alternate 0 pull-down).
 * PA15 - DWM_SPI_CSn               (output push-pull).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_VBAT_PROBE) |    \
                                     PIN_MODE_OUTPUT(GPIOA_MTR_LED_L)   |   \
                                     PIN_MODE_ALTERNATE(GPIOA_MTR_PHASE_L) |\
                                     PIN_MODE_ALTERNATE(GPIOA_MTR_ENABLE_L)|\
                                     PIN_MODE_INPUT(GPIOA_PIN4) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_MTR_PHASE_R) |\
                                     PIN_MODE_ALTERNATE(GPIOA_MTR_ENABLE_R)|\
                                     PIN_MODE_ANALOG(GPIOA_MTR_TRA_L) |     \
                                     PIN_MODE_INPUT(GPIOA_PIN8) |           \
                                     PIN_MODE_ANALOG(GPIOA_USB_VBUS) |      \
                                     PIN_MODE_OUTPUT(GPIOA_USB_CONNECT) |   \
                                     PIN_MODE_ANALOG(GPIOA_USB_DM) |        \
                                     PIN_MODE_ANALOG(GPIOA_USB_DP) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_OUTPUT(GPIOA_DWM_SPI_CSn))
#define VAL_GPIOA_OTYPER    0x00000000
#define VAL_GPIOA_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLUP(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_PULLDOWN(GPIOA_SWCLK)|       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_MTR_PHASE_L)| \
                                     PIN_PUPDR_PULLDOWN(GPIOA_MTR_ENABLE_L)|\
                                     PIN_PUPDR_PULLDOWN(GPIOA_MTR_PHASE_R)| \
                                     PIN_PUPDR_PULLDOWN(GPIOA_MTR_ENABLE_R))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_DWM_SPI_CSn)|       \
                                     PIN_ODR_HIGH(GPIOA_MTR_LED_L))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_MTR_ENABLE_L, 9U) |  \
                                     PIN_AFIO_AF(GPIOA_MTR_PHASE_L, 9U) |   \
                                     PIN_AFIO_AF(GPIOA_MTR_PHASE_R, 1U) |   \
                                     PIN_AFIO_AF(GPIOA_MTR_ENABLE_R, 1U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - MTR_TRA_R                 (analog).
 * PB1  - LED_IR_C                  (output push-pull).
 * PB2  - PIN2                      (input floating).
 * PB3  - DWM_WAKEUP                (input floating).
 * PB4  - PIN4                      (input floating).
 * PB5  - PIN5                      (input floating).
 * PB6  - PIN6                      (input floating).
 * PB7  - MPU_INT                   (input pull-down).
 * PB8  - MPU_I2C_SCL               (alternate 4 open drain).
 * PB9  - MPU_I2C_SDA               (alternate 4 open drain).
 * PB10 - LED_IR_R                  (output push-pull).
 * PB11 - LED_IR_L                  (output push-pull).
 * PB12 - PIN12                     (input floating).
 * PB13 - LED_SPI_CK                (alternate 5).
 * PB14 - PIN14                     (input floating).
 * PB15 - LED_SPI_DO                (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_MTR_TRA_R) |     \
                                     PIN_MODE_ALTERNATE(GPIOB_LED_IR_C) |   \
                                     PIN_MODE_INPUT(GPIOB_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOB_DWM_WAKEUP) |    \
                                     PIN_MODE_INPUT(GPIOB_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOB_MPU_INT) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_MPU_I2C_SCL) |\
                                     PIN_MODE_ALTERNATE(GPIOB_MPU_I2C_SDA) |\
                                     PIN_MODE_OUTPUT(GPIOB_LED_IR_R) |      \
                                     PIN_MODE_OUTPUT(GPIOB_LED_IR_L) |      \
                                     PIN_MODE_INPUT(GPIOB_PIN12) |          \
                                     PIN_MODE_ALTERNATE(GPIOB_LED_SPI_CK) | \
                                     PIN_MODE_INPUT(GPIOB_PIN14) |          \
                                     PIN_MODE_ALTERNATE(GPIOB_LED_SPI_DO))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_MPU_I2C_SCL) |\
                                     PIN_OTYPE_OPENDRAIN(GPIOB_MPU_I2C_SDA))
#define VAL_GPIOB_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLDOWN(GPIOB_MPU_INT))
#define VAL_GPIOB_ODR       0x00000000
#define VAL_GPIOB_AFRL               PIN_AFIO_AF(GPIOB_LED_IR_C, 8U)
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_MPU_I2C_SCL, 4U) |   \
                                     PIN_AFIO_AF(GPIOB_MPU_I2C_SDA, 4U) |   \
                                     PIN_AFIO_AF(GPIOB_LED_SPI_CK, 5U)  |   \
                                     PIN_AFIO_AF(GPIOB_LED_SPI_DO, 5U))

/*
 * GPIOC setup:
 *
 * PC13 - PIN13                     (input floating).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
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
 * PF0  - OSC_IN                    (input floating).
 * PF1  - OSC_OUT                   (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOF_OSC_OUT))
#define VAL_GPIOF_OTYPER    0x00000000
#define VAL_GPIOF_OSPEEDR   0xFFFFFFFF
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_OSC_OUT))
#define VAL_GPIOF_ODR       0x00000000
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_OSC_IN, 0U)  |       \
                                     PIN_AFIO_AF(GPIOF_OSC_OUT, 0U))
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

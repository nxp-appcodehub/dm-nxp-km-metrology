/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief
 * UART0 IrDA Select: Pad RX input (PTD[0], PTF[3] or PTK[3], as selected in Pinmux control) selected for RX input
 * of UART0 and UART0 TX signal is not used for modulation.
 */
#define MISC_CTL_UART0IRSEL_0b0 0x00u

/*! @name PORTI0 (number ), UART1_RX
  @{ */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_UART1_RX_PORT PORTI               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_UART1_RX_PIN 0U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_UART1_RX_PIN_MASK (1U << 0U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PORTI1 (number ), UART1_TX
  @{ */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_UART1_TX_PORT PORTI               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_UART1_TX_PIN 1U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_UART1_TX_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PORTK3 (number 82), UART0_RX
  @{ */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_UART0_RX_PORT PORTK               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_UART0_RX_PIN 3U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_UART0_RX_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PORTK2 (number 81), UART0_TX
  @{ */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_UART0_TX_PORT PORTK               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_UART0_TX_PIN 2U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_UART0_TX_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PORTA4 (number 10), SW1
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_SW1_FGPIO FGPIOA             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_SW1_GPIO GPIOA               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_SW1_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_SW1_PORT PORTA               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SW1_PIN 4U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SW1_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                                    /* @} */

/*! @name PORTD1 (number 64), SW2
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_SW2_FGPIO FGPIOD             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_SW2_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_SW2_GPIO_PIN_MASK (1U << 1U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_SW2_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SW2_PIN 1U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SW2_PIN_MASK (1U << 1U)      /*!<@brief PORT pin mask */
                                                    /* @} */

/*! @name PORTJ4 (number 63), LED_RED
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_LED_RED_FGPIO FGPIOJ             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_LED_RED_GPIO GPIOJ               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_LED_RED_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_LED_RED_PORT PORTJ               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_LED_RED_PIN 4U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_LED_RED_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                                        /* @} */

/*! @name PORTJ3 (number 62), LED_GREEN
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_LED_GREEN_FGPIO FGPIOJ             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_LED_GREEN_GPIO GPIOJ               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK (1U << 3U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_LED_GREEN_PORT PORTJ               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_LED_GREEN_PIN 3U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_LED_GREEN_PIN_MASK (1U << 3U)      /*!<@brief PORT pin mask */
                                                          /* @} */

/*! @name PORTD0 (number 61), LED_ORANGE
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_LED_ORANGE_FGPIO FGPIOD             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_LED_ORANGE_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK (1U << 0U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_LED_ORANGE_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_LED_ORANGE_PIN 0U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_LED_ORANGE_PIN_MASK (1U << 0U)      /*!<@brief PORT pin mask */
                                                           /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/

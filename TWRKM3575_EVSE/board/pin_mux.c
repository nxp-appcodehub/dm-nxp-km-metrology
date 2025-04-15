/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v9.0
processor: MKM35Z512xxx7
package_id: MKM35Z512VLQ7
mcu_data: ksdk2_0
processor_version: 9.0.1
pin_labels:
- {pin_num: '10', pin_signal: LCD_P27/PTA4/LLWU_P15/NMI_b, label: SW1, identifier: SW1}
- {pin_num: '63', pin_signal: PTJ4/LPUART0_CTS_b/LPTMR1_ALT1, label: LED_RED, identifier: LED_RED}
- {pin_num: '62', pin_signal: PTJ3/LPUART0_RTS_b/CMP2_OUT, label: LED_GREEN, identifier: LED_GREEN}
- {pin_num: '61', pin_signal: CMP0_IN0/PTD0/LLWU_P11/UART0_RX/XBAR_IN2, label: LED_ORANGE, identifier: LED_ORANGE}
- {pin_num: '64', pin_signal: PTD1/UART1_TX/SPI0_PCS0/XBAR_OUT3/QTMR0_TMR3, label: SW2, identifier: SW2}
- {pin_num: '82', pin_signal: ADC0_SE15/PTK3/LLWU_P19/UART0_RX, label: UART0_RX, identifier: UART0_RX}
- {pin_num: '81', pin_signal: ADC0_SE14/PTK2/UART0_TX, label: UART0_TX, identifier: UART0_TX}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '4', peripheral: LCD, signal: 'P, 23', pin_signal: LCD_P23/PTA0/LLWU_P16}
  - {pin_num: '5', peripheral: LCD, signal: 'P, 24', pin_signal: LCD_P24/PTA1}
  - {pin_num: '8', peripheral: LCD, signal: 'P, 25', pin_signal: LCD_P25/PTA2}
  - {pin_num: '9', peripheral: LCD, signal: 'P, 26', pin_signal: LCD_P26/PTA3}
  - {pin_num: '11', peripheral: LCD, signal: 'P, 28', pin_signal: LCD_P28/PTA5/CMP0_OUT}
  - {pin_num: '12', peripheral: LCD, signal: 'P, 29', pin_signal: LCD_P29/PTA6/LLWU_P14/XBAR_IN0}
  - {pin_num: '13', peripheral: LCD, signal: 'P, 30', pin_signal: LCD_P30/PTA7/XBAR_OUT0}
  - {pin_num: '16', peripheral: LCD, signal: 'P, 31', pin_signal: LCD_P31/PTB0}
  - {pin_num: '20', peripheral: LCD, signal: 'P, 32', pin_signal: LCD_P32/PTB1/LLWU_P17}
  - {pin_num: '21', peripheral: LCD, signal: 'P, 33', pin_signal: LCD_P33/PTB2/SPI2_PCS0}
  - {pin_num: '22', peripheral: LCD, signal: 'P, 34', pin_signal: LCD_P34/PTB3/SPI2_SCK}
  - {pin_num: '23', peripheral: LCD, signal: 'P, 35', pin_signal: LCD_P35/PTB4/SPI2_MISO}
  - {pin_num: '24', peripheral: LCD, signal: 'P, 36', pin_signal: LCD_P36/PTB5/SPI2_MOSI}
  - {pin_num: '25', peripheral: LCD, signal: 'P, 37', pin_signal: LCD_P37/CMP1_IN0/PTB6}
  - {pin_num: '26', peripheral: LCD, signal: 'P, 38', pin_signal: LCD_P38/PTB7/AFE_CLK}
  - {pin_num: '31', peripheral: LCD, signal: 'P, 43', pin_signal: LCD_P43/PTC4}
  - {pin_num: '116', peripheral: LCD, signal: 'P, 13', pin_signal: LCD_P13/PTG6/LLWU_P0/LPTMR0_ALT3}
  - {pin_num: '117', peripheral: LCD, signal: 'P, 14', pin_signal: LCD_P14/PTG7/LPTMR1_ALT1}
  - {pin_num: '122', peripheral: LCD, signal: 'P, 19', pin_signal: LCD_P19/PTH4/LPTMR1_ALT2}
  - {pin_num: '123', peripheral: LCD, signal: 'P, 20', pin_signal: LCD_P20/PTH5/LPTMR1_ALT3}
  - {pin_num: '131', peripheral: LCD, signal: 'P, 22', pin_signal: LCD_P22/PTI3/LPUART0_TX/CMP2_OUT}
  - {pin_num: '143', peripheral: LCD, signal: 'P, 44', pin_signal: LCD_P44/PTI4}
  - {pin_num: '3', peripheral: LCD, signal: 'P, 45', pin_signal: LCD_P45/PTI5}
  - {pin_num: '17', peripheral: LCD, signal: 'P, 50', pin_signal: LCD_P50/PTJ2}
  - {pin_num: '139', peripheral: LCD, signal: 'P, 56', pin_signal: LCD_P56/PTL3/EWM_IN}
  - {pin_num: '140', peripheral: LCD, signal: 'P, 57', pin_signal: LCD_P57/PTL4/EWM_OUT_b}
  - {pin_num: '141', peripheral: LCD, signal: 'P, 58', pin_signal: LCD_P58/PTL5/LLWU_P23}
  - {pin_num: '142', peripheral: LCD, signal: 'P, 59', pin_signal: LCD_P59/PTL6}
  - {pin_num: '95', peripheral: ADC0, signal: 'SE, 8', pin_signal: LCD_P0/ADC0_SE8/CMP2_IN4/PTF1/QTMR0_TMR0/XBAR_OUT6}
  - {pin_num: '82', peripheral: UART0, signal: RX, pin_signal: ADC0_SE15/PTK3/LLWU_P19/UART0_RX, pull_select: up, pull_enable: enable}
  - {pin_num: '81', peripheral: UART0, signal: TX, pin_signal: ADC0_SE14/PTK2/UART0_TX, direction: OUTPUT, pull_select: up, pull_enable: enable}
  - {pin_num: '110', peripheral: TMR1, signal: OUT, pin_signal: LCD_P7/PTG0/QTMR0_TMR1/LPTMR0_ALT3}
  - {pin_num: '10', peripheral: GPIOA, signal: 'GPIO, 4', pin_signal: LCD_P27/PTA4/LLWU_P15/NMI_b, direction: INPUT, gpio_interrupt: kPORT_InterruptFallingEdge}
  - {pin_num: '64', peripheral: GPIOD, signal: 'GPIO, 1', pin_signal: PTD1/UART1_TX/SPI0_PCS0/XBAR_OUT3/QTMR0_TMR3, direction: INPUT, gpio_interrupt: kPORT_InterruptFallingEdge,
    pull_select: up, pull_enable: enable}
  - {pin_num: '63', peripheral: GPIOJ, signal: 'GPIO, 4', pin_signal: PTJ4/LPUART0_CTS_b/LPTMR1_ALT1, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '62', peripheral: GPIOJ, signal: 'GPIO, 3', pin_signal: PTJ3/LPUART0_RTS_b/CMP2_OUT, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: '61', peripheral: GPIOD, signal: 'GPIO, 0', pin_signal: CMP0_IN0/PTD0/LLWU_P11/UART0_RX/XBAR_IN2, direction: OUTPUT, gpio_init_state: 'true'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* PCTLA Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* PCTLB Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* PCTLC Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* PCTLD Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* PCTLF Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortF);
    /* PCTLG Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortG);
    /* PCTLH Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortH);
    /* PCTLI Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortI);
    /* PCTLJ Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortJ);
    /* PCTLK Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortK);
    /* PCTLL Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortL);

    gpio_pin_config_t SW1_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA4 (pin 10)  */
    GPIO_PinInit(BOARD_INITPINS_SW1_GPIO, BOARD_INITPINS_SW1_PIN, &SW1_config);

    gpio_pin_config_t LED_ORANGE_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTD0 (pin 61)  */
    GPIO_PinInit(BOARD_INITPINS_LED_ORANGE_GPIO, BOARD_INITPINS_LED_ORANGE_PIN, &LED_ORANGE_config);

    gpio_pin_config_t SW2_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTD1 (pin 64)  */
    GPIO_PinInit(BOARD_INITPINS_SW2_GPIO, BOARD_INITPINS_SW2_PIN, &SW2_config);

    gpio_pin_config_t LED_GREEN_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTJ3 (pin 62)  */
    GPIO_PinInit(BOARD_INITPINS_LED_GREEN_GPIO, BOARD_INITPINS_LED_GREEN_PIN, &LED_GREEN_config);

    gpio_pin_config_t LED_RED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTJ4 (pin 63)  */
    GPIO_PinInit(BOARD_INITPINS_LED_RED_GPIO, BOARD_INITPINS_LED_RED_PIN, &LED_RED_config);

    /* PORTA0 (pin 4) is configured as LCD_P23 */
    PORT_SetPinMux(PORTA, 0U, kPORT_PinDisabledOrAnalog);

    /* PORTA1 (pin 5) is configured as LCD_P24 */
    PORT_SetPinMux(PORTA, 1U, kPORT_PinDisabledOrAnalog);

    /* PORTA2 (pin 8) is configured as LCD_P25 */
    PORT_SetPinMux(PORTA, 2U, kPORT_PinDisabledOrAnalog);

    /* PORTA3 (pin 9) is configured as LCD_P26 */
    PORT_SetPinMux(PORTA, 3U, kPORT_PinDisabledOrAnalog);

    /* PORTA4 (pin 10) is configured as PTA4 */
    PORT_SetPinMux(BOARD_INITPINS_SW1_PORT, BOARD_INITPINS_SW1_PIN, kPORT_MuxAsGpio);

    /* Interrupt configuration on PORTA4 (pin 10): Interrupt on falling edge */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_SW1_PORT, BOARD_INITPINS_SW1_PIN, kPORT_InterruptFallingEdge);

    /* PORTA5 (pin 11) is configured as LCD_P28 */
    PORT_SetPinMux(PORTA, 5U, kPORT_PinDisabledOrAnalog);

    /* PORTA6 (pin 12) is configured as LCD_P29 */
    PORT_SetPinMux(PORTA, 6U, kPORT_PinDisabledOrAnalog);

    /* PORTA7 (pin 13) is configured as LCD_P30 */
    PORT_SetPinMux(PORTA, 7U, kPORT_PinDisabledOrAnalog);

    /* PORTB0 (pin 16) is configured as LCD_P31 */
    PORT_SetPinMux(PORTB, 0U, kPORT_PinDisabledOrAnalog);

    /* PORTB1 (pin 20) is configured as LCD_P32 */
    PORT_SetPinMux(PORTB, 1U, kPORT_PinDisabledOrAnalog);

    /* PORTB2 (pin 21) is configured as LCD_P33 */
    PORT_SetPinMux(PORTB, 2U, kPORT_PinDisabledOrAnalog);

    /* PORTB3 (pin 22) is configured as LCD_P34 */
    PORT_SetPinMux(PORTB, 3U, kPORT_PinDisabledOrAnalog);

    /* PORTB4 (pin 23) is configured as LCD_P35 */
    PORT_SetPinMux(PORTB, 4U, kPORT_PinDisabledOrAnalog);

    /* PORTB5 (pin 24) is configured as LCD_P36 */
    PORT_SetPinMux(PORTB, 5U, kPORT_PinDisabledOrAnalog);

    /* PORTB6 (pin 25) is configured as LCD_P37 */
    PORT_SetPinMux(PORTB, 6U, kPORT_PinDisabledOrAnalog);

    /* PORTB7 (pin 26) is configured as LCD_P38 */
    PORT_SetPinMux(PORTB, 7U, kPORT_PinDisabledOrAnalog);

    /* PORTC4 (pin 31) is configured as LCD_P43 */
    PORT_SetPinMux(PORTC, 4U, kPORT_PinDisabledOrAnalog);

    /* PORTD0 (pin 61) is configured as PTD0 */
    PORT_SetPinMux(BOARD_INITPINS_LED_ORANGE_PORT, BOARD_INITPINS_LED_ORANGE_PIN, kPORT_MuxAsGpio);

    /* PORTD1 (pin 64) is configured as PTD1 */
    PORT_SetPinMux(BOARD_INITPINS_SW2_PORT, BOARD_INITPINS_SW2_PIN, kPORT_MuxAsGpio);

    /* Interrupt configuration on PORTD1 (pin 64): Interrupt on falling edge */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_SW2_PORT, BOARD_INITPINS_SW2_PIN, kPORT_InterruptFallingEdge);

    PORTD->PCR[1] = ((PORTD->PCR[1] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                      * corresponding PE field is set. */
                     | (uint32_t)(kPORT_PullUp));

    /* PORTF1 (pin 95) is configured as ADC0_SE8 */
    PORT_SetPinMux(PORTF, 1U, kPORT_PinDisabledOrAnalog);

    /* PORTG0 (pin 110) is configured as QTMR0_TMR1 */
    PORT_SetPinMux(PORTG, 0U, kPORT_MuxAlt2);

    /* PORTG6 (pin 116) is configured as LCD_P13 */
    PORT_SetPinMux(PORTG, 6U, kPORT_PinDisabledOrAnalog);

    /* PORTG7 (pin 117) is configured as LCD_P14 */
    PORT_SetPinMux(PORTG, 7U, kPORT_PinDisabledOrAnalog);

    /* PORTH4 (pin 122) is configured as LCD_P19 */
    PORT_SetPinMux(PORTH, 4U, kPORT_PinDisabledOrAnalog);

    /* PORTH5 (pin 123) is configured as LCD_P20 */
    PORT_SetPinMux(PORTH, 5U, kPORT_PinDisabledOrAnalog);

    /* PORTI3 (pin 131) is configured as LCD_P22 */
    PORT_SetPinMux(PORTI, 3U, kPORT_PinDisabledOrAnalog);

    /* PORTI4 (pin 143) is configured as LCD_P44 */
    PORT_SetPinMux(PORTI, 4U, kPORT_PinDisabledOrAnalog);

    /* PORTI5 (pin 3) is configured as LCD_P45 */
    PORT_SetPinMux(PORTI, 5U, kPORT_PinDisabledOrAnalog);

    /* PORTJ2 (pin 17) is configured as LCD_P50 */
    PORT_SetPinMux(PORTJ, 2U, kPORT_PinDisabledOrAnalog);

    /* PORTJ3 (pin 62) is configured as PTJ3 */
    PORT_SetPinMux(BOARD_INITPINS_LED_GREEN_PORT, BOARD_INITPINS_LED_GREEN_PIN, kPORT_MuxAsGpio);

    /* PORTJ4 (pin 63) is configured as PTJ4 */
    PORT_SetPinMux(BOARD_INITPINS_LED_RED_PORT, BOARD_INITPINS_LED_RED_PIN, kPORT_MuxAsGpio);

    /* PORTK2 (pin 81) is configured as UART0_TX */
    PORT_SetPinMux(BOARD_INITPINS_UART0_TX_PORT, BOARD_INITPINS_UART0_TX_PIN, kPORT_MuxAlt2);

    PORTK->PCR[2] = ((PORTK->PCR[2] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                      * corresponding PE field is set. */
                     | (uint32_t)(kPORT_PullUp));

    /* PORTK3 (pin 82) is configured as UART0_RX */
    PORT_SetPinMux(BOARD_INITPINS_UART0_RX_PORT, BOARD_INITPINS_UART0_RX_PIN, kPORT_MuxAlt2);

    PORTK->PCR[3] = ((PORTK->PCR[3] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Select: Internal pullup resistor is enabled on the corresponding pin, if the
                      * corresponding PE field is set. */
                     | (uint32_t)(kPORT_PullUp));

    /* PORTK4 (pin 85) is configured as AFE_CLK */
	PORT_SetPinMux(PORTK, 4U, kPORT_MuxAlt3);

	/* PORTL3 (pin 129) is configured as AFE Data Ready */
	PORT_SetPinMux(PORTL, 2U, kPORT_MuxAsGpio);

	/* SPI1 pins enable */
	PORT_SetPinMux(PORTF, 3U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTF, 4U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTF, 5U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTF, 6U, kPORT_MuxAlt2);

    /* PORTL3 (pin 139) is configured as LCD_P56 */
    PORT_SetPinMux(PORTL, 3U, kPORT_PinDisabledOrAnalog);

    /* PORTL4 (pin 140) is configured as LCD_P57 */
    PORT_SetPinMux(PORTL, 4U, kPORT_PinDisabledOrAnalog);

    /* PORTL5 (pin 141) is configured as LCD_P58 */
    PORT_SetPinMux(PORTL, 5U, kPORT_PinDisabledOrAnalog);

    /* PORTL6 (pin 142) is configured as LCD_P59 */
    PORT_SetPinMux(PORTL, 6U, kPORT_PinDisabledOrAnalog);

    SIM->MISC_CTL = ((SIM->MISC_CTL &
                      /* Mask bits to zero which are setting */
                      (~(SIM_MISC_CTL_AFECLKPADDIR_MASK | SIM_MISC_CTL_UART0IRSEL_MASK)))
                     /* UART0 IrDA Select: Pad RX input (PTD[0], PTF[3] or PTK[3], as selected in Pinmux control)
                      * selected for RX input of UART0 and UART0 TX signal is not used for modulation. */
                     | SIM_MISC_CTL_UART0IRSEL(MISC_CTL_UART0IRSEL_0b0));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/

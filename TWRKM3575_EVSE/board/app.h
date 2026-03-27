/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*${header:start}*/
#include "pin_mux.h"
/*${header:end}*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define BOARD_LED1_GPIO     BOARD_LED_kWh_GPIO
#define BOARD_LED1_GPIO_PIN BOARD_LED_kWh_PIN
#define BOARD_LED2_GPIO     BOARD_LED_kVARh_GPIO
#define BOARD_LED2_GPIO_PIN BOARD_LED_kVARh_PIN
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */

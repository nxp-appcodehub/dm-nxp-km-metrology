/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022, 2024, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include <stdio.h>
#include <string.h>

#include "fsl_adc16.h"
#include "fsl_afe.h"
#include "fsl_clock.h"
#include "fsl_lptmr.h"
#include "fsl_port.h"
#include "fsl_qtmr.h"
#include "fsl_slcd.h"
#include "fsl_vref.h"
#include "fsl_xbar.h"

#include "slcd_engine.h"

#include "fraclib.h"
#include "meterlib.h"
#include "meterlib1ph_cfg.h"
#include <math.h>

#include "fsl_crc.h"
#include "fsl_dmamux.h"
#include "fsl_spi_dma.h"
#include "fsl_xbar.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define offLow32Pin(n)                                                         \
  ((uint32_t)1 << (n)) /* Pin offset for the low 32 pins. */
#define offHigh32Pin(n)                                                        \
  ((uint32_t)1 << (n - 32)) /* Pin offset for the high 32 pins. */

#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_IRQN ADC0_IRQn
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 8U

/* Low Power Timer */
#define LPTMR_CLK_FREQ CLOCK_GetFreq(kCLOCK_LpoClk)
#define LPTMR0_IRQHandler LPTMR0_LPTMR1_IRQHandler

#define PI 3.14159265358979323846 /* pi */

#define BUS_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)

/* GPIO and SW1 */
#define BOARD_SW_IRQ BOARD_SW1_IRQ

#define SARADCCallback ADC0_IRQHandler

#define SPI_SLAVE SPI1
#define DMA DMA0
/* #define DMAMUX           DMAMUX */
#define SPI_TX_CHANNEL 3U
#define SPI_RX_CHANNEL 2U
#define SPI_TX_SOURCE kDmaRequestMux0SPI1Tx
#define SPI_RX_SOURCE kDmaRequestMux0SPI1Rx

#define SAMPLE_SIZE 64U /* Number of samples per sine wave cycle: 64 or 120 */

#define I_MAX_BRD 32 /* Supported max. AC current by the meter board */

/*-------  Version Numbering of the TWR-KM35Z75M code -------*/
#define MAJOR_VER 1 /* Major Release number */
#define MINOR_VER 2 /* Minor Release number*/
#define BUG_VER 0

#define CONV_STR(s) #s
#define DEF_TO_STR(s) CONV_STR(s)

#define TWRKM35Z75M_VERSION                                                    \
  DEF_TO_STR(MAJOR_VER) "." DEF_TO_STR(MINOR_VER) "." DEF_TO_STR(BUG_VER)

typedef enum { kHWTriggerMode, kSWTriggerMode } adc_trig_mode_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*
 * PWM control used to indicate EVSE current capability to vehicle
 * onoff = 0 then PWM output disabled
 * onoff = 1 then PWM output enabled
 * duty = on time as percentage
 * Only enabled during charge phase
 */
static void PWM_OnOFF(uint8_t onoff, uint8_t perc);
/*!
 * @brief SLCD set lcd pins.
 *
 * @param type lcd setting type @ref "lcd_set_type_t".
 * @param lcd_pin lcd pin.
 * @param pin_val pin setting value.
 * @param on The display on/off flag.
 */
static void SLCD_SetLCDPin(lcd_set_type_t type, uint32_t lcd_pin,
                           uint8_t pin_val, int32_t on);

/*!
 * @brief SLCD Application Initialization.
 */
static void SLCD_APP_Init(void);

/*!
 * @brief SLCD Clear Screen.
 */
static void SLCD_Clear_Screen(void);

/*
 * @brief This handler is used to set dummy data in current sense buffers.
 */
static void Initialize_Sample_Buffer(void);

/*
 * @brief Initialize SAR ADC channels
 */
static void InitSARADC(adc_trig_mode_t trigger_mode);

/*!
 * @brief Initializes the MCU resources used to aid the core metrology
 * calculation.
 */
static void MeteringInit(void);

/*
 * @brief This handler is used to update the current sense buffer with the
 * potentiometer adjustment.
 */
static void Update_current_sense_data(void);

/*
 * @brief This function is used to connect AFE channel conversion complete flags
 * to SAR-ADC triggers using XBAR.
 */
static void ConnectAFEtoSARADC(void);

/*!
 * @brief Initializes AFE channels for voltage and current channels.
 */
static void InitAFE(void);

/*
 * @brief This handler is used to CRC value of a data buffer.
 */
static uint32_t GetCRC(uint32_t *data, uint32_t size);

/*
 * @brief This function is used to simulate a small delay.
 */
static void delay(void);

/*
 * @brief This handler is used to prepare a data packet with header, data and
 * CRC value.
 */
static void PrepareSPIPacket(void);

/*!
 * @brief Iniitializes VREF module so that the reference voltage can be used by
 * other modules(e.g., Comparator with line voltage to generate an o/p to
 * triggger the timer for frequency calculation.
 */
static void InitVREF(void);

/*!
 *@ SPI DMA Init
 */
static void SPI_DMA_init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t tmp32_2;
volatile static uint16_t tmp16;
static adc16_channel_config_t adc16ChannelConfigStruct;
static float Pot_Value; // Potentiometer reading on TWR board scaled to 32A
static bool read_pot_val = true;

static tSLCD_Engine slcd_engine;

static volatile frac32 u24_sample, i24_sample;
static tENERGY_CNT wh_cnt, varh_cnt;
static double U_RMS, I_RMS, P, Q, S;
static frac16 shift = METERLIB_DEG2SH(-5.5, 50.0);
static uint8_t State_Counter =
    0; /* States are 1: State A Not Connected 2: State B Connected Ready 3:
          State C Charging 4: Vent*/
static uint8_t Error_Counter =
    0; /* States are 0: no error 1: State E Error 2: State F Error */

/* Whether the SW button is pressed */
volatile bool g_Button1Press = false;
static volatile uint8_t led_turn = 0;

static volatile bool PWMRunning = false;

static char Status_String[7] = "STATEA";
static float Status_Index = 1.0;

/* SPI-DMA variables */
static uint32_t ui_buff[6];
uint32_t dummy;
#define BUFFER_SIZE (4 + 24 + 4) /*  4B command + 24B data + 4B CRC */
static uint8_t buff[BUFFER_SIZE];
static uint8_t sendBuff[BUFFER_SIZE];
static spi_dma_handle_t s_handle;
static dma_handle_t txHandle;
static dma_handle_t rxHandle;
static volatile bool slaveFinished = false;
static int sample_index = 0;
static int32_t current_sense_buf[3][SAMPLE_SIZE] = {0};
bool volatile last_transfer_done = true;
CRC_Type *base = CRC0;
uint8_t afe_inactivite_counter = 0;
static uint16_t adc_hwtrigger_counter = false;
static bool adc_hwtrigger_detected = false;
static bool afe_run_failed = false;

#if (SAMPLE_SIZE == 64)
/*  sinus: magnitude=6e6, 64 samples, 50Hz (used for OSR=1024,AFECLK=6.144MHz),
    5-th harmonics with ratio 10% */
static frac24 sin_64s_6e6_5h_10p[] = {
    0,        870941,   1669424,  2338819,  2850428,  3209016,  3450476,
    3632189,  3818377,  4063899,  4400347,  4827721,  5313667,  5800452,
    6218054,  6500261,  6600000,  6500261,  6218054,  5800452,  5313667,
    4827721,  4400347,  4063899,  3818377,  3632189,  3450476,  3209016,
    2850428,  2338819,  1669424,  870941,   0,        -870941,  -1669424,
    -2338819, -2850428, -3209016, -3450476, -3632189, -3818377, -4063899,
    -4400347, -4827721, -5313667, -5800452, -6218054, -6500261, -6600000,
    -6500261, -6218054, -5800452, -5313667, -4827721, -4400347, -4063899,
    -3818377, -3632189, -3450476, -3209016, -2850428, -2338819, -1669424,
    -870941};

/*  sinus with shift: magnitude=4e6, 64 samples, 50Hz (used for
   OSR=1024,AFECLK=6.144MHz), 5-th harmonics with ratio 40% and shift 4.5
   degree, 1-st harmonic shift is 5.3 degree, DC offset = 600000 */
frac24 sin_64s_4e6_5h_40p_sh_offs[] = {
    1095017,  2220717,  3135401,  3709439,  3891157,  3718241,  3308053,
    2829185,  2460949,  2350394,  2577004,  3133410,  3926686,  4799926,
    5569001,  6065836,  6177967,  5875052,  5216080,  4335555,  3411939,
    2625788,  2117476,  1954481,  2115954,  2498216,  2939871,  3260611,
    3304523,  2977684,  2271149,  1263887,  104983,   -1020717, -1935401,
    -2509439, -2691157, -2518241, -2108053, -1629185, -1260949, -1150394,
    -1377004, -1933410, -2726686, -3599926, -4369001, -4865836, -4977967,
    -4675052, -4016080, -3135555, -2211939, -1425788, -917476,  -754481,
    -915954,  -1298216, -1739871, -2060611, -2104523, -1777684, -1071149,
    -63887};

#endif

#if (SAMPLE_SIZE == 120)
long sin_120s_6e6_5h_10p[120] = {
    0,        469307,   927171,   1362871,  1767085,  2132470,  2454102,
    2729763,  2960035,  3148207,  3300000,  3423126,  3526712,  3620631,
    3714784,  3818377,  3939254,  4083320,  4254102,  4452468,  4676537,
    4921775,  5181273,  5446191,  5706339,  5950846,  6168886,  6350394,
    6486747,  6571333,  6600000,  6571333,  6486747,  6350394,  6168886,
    5950846,  5706339,  5446191,  5181273,  4921775,  4676537,  4452468,
    4254102,  4083320,  3939254,  3818377,  3714784,  3620631,  3526712,
    3423126,  3300000,  3148207,  2960035,  2729763,  2454102,  2132470,
    1767085,  1362871,  927171,   469307,   0,        -469307,  -927171,
    -1362871, -1767085, -2132470, -2454102, -2729763, -2960035, -3148207,
    -3300000, -3423126, -3526712, -3620631, -3714784, -3818377, -3939254,
    -4083320, -4254102, -4452468, -4676537, -4921775, -5181273, -5446191,
    -5706339, -5950846, -6168886, -6350394, -6486747, -6571333, -6600000,
    -6571333, -6486747, -6350394, -6168886, -5950846, -5706339, -5446191,
    -5181273, -4921775, -4676537, -4452468, -4254102, -4083320, -3939254,
    -3818377, -3714784, -3620631, -3526712, -3423126, -3300000, -3148207,
    -2960035, -2729763, -2454102, -2132470, -1767085, -1362871, -927171,
    -469307};

long sin_120s_4e6_5h_40p_sh_offs[120] = {
    1095017,  1711516,  2290035,  2804646,  3233636,  3560950,  3777250,
    3880511,  3876132,  3776526,  3600248,  3370693,  3114472,  2859570,
    2633408,  2460949,  2362966,  2354609,  2444341,  2633348,  2915431,
    3277413,  3700023,  4159190,  4627673,  5076905,  5478933,  5808312,
    6043838,  6169985,  6177967,  6066330,  5841061,  5515180,  5107861,
    4643133,  4148251,  3651855,  3182027,  2764398,  2420414,  2165898,
    2009989,  1954545,  1994037,  2115954,  2301683,  2527801,  2767706,
    2993459,  3177718,  3295637,  3326588,  3255610,  3074453,  2782184,
    2385285,  1897246,  1337686,  731050,   104983,   -511516,  -1090035,
    -1604646, -2033636, -2360950, -2577250, -2680511, -2676132, -2576526,
    -2400248, -2170693, -1914472, -1659570, -1433408, -1260949, -1162966,
    -1154609, -1244341, -1433348, -1715431, -2077413, -2500023, -2959190,
    -3427673, -3876905, -4278933, -4608312, -4843838, -4969985, -4977967,
    -4866330, -4641061, -4315180, -3907861, -3443133, -2948251, -2451855,
    -1982027, -1564398, -1220414, -965898,  -809989,  -754545,  -794037,
    -915954,  -1101683, -1327801, -1567706, -1793459, -1977718, -2095637,
    -2126588, -2055610, -1874453, -1582184, -1185285, -697246,  -137686,
    468950};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Interrupt service function of switch.
 *
 * This function is called when SW1 or SW2 is pressed
 *  */
void BOARD_SW1_IRQ_HANDLER(void) {
  /* Check which SW was pressed by testing each port ISR register */
  if (GPIO_PortGetInterruptFlags(BOARD_SW1_GPIO) & (1U << BOARD_SW1_GPIO_PIN)) {
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW1_GPIO, 1U << BOARD_SW1_GPIO_PIN);
    /* Send Status. */
    switch (State_Counter) {
    case 0:
      strcpy(Status_String, "STATEA");
      Status_Index = 1.0;
      PRINTF("%1.1f[4]\r", Status_Index);
      PWM_OnOFF(0, 53);
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      g_Button1Press = false;
      Error_Counter = 0;
      break;
    case 1:
      strcpy(Status_String, "STATEB");
      Status_Index = 2.0;
      PRINTF("%1.1f[4]\r", Status_Index);
      PWM_OnOFF(0, 53);
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_GREEN_GPIO,
                     BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      g_Button1Press = false;
      break;
    case 2:
      strcpy(Status_String, "STATEC");
      Status_Index = 3.0;
      PRINTF("%1.1f[4]\r", Status_Index);
      PWM_OnOFF(1, 53);
      /* Change state of button. */
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      led_turn = 0;
      g_Button1Press = true;
      break;
    case 3:
      strcpy(Status_String, "STATED");
      Status_Index = 4.0;
      PRINTF("%1.1f[4]\r", Status_Index);
      PWM_OnOFF(0, 53);
      /* Change state of button. */
      GPIO_PortClear(BOARD_INITPINS_LED_ORANGE_GPIO,
                     BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_GREEN_GPIO,
                     BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      g_Button1Press = false;
      break;
    default:
      break;
    }
    State_Counter++;
    State_Counter %= 4U;
  }
  if (GPIO_PortGetInterruptFlags(BOARD_SW2_GPIO) & (1U << BOARD_SW2_GPIO_PIN)) {
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);
    /* Change state of button. */
    /* reset charge state */
    State_Counter = 0;
    Error_Counter++;
    if (Error_Counter > 2U) {
      Error_Counter = 0;
    }
    /* Send Status. */
    switch (Error_Counter) {
    case 0:
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      break;
    case 1:
      strcpy(Status_String, "STATEE");
      Status_Index = 5.0;
      PRINTF("%1.1f[4]\r", Status_Index);
      PWM_OnOFF(0, 80);
      GPIO_PortClear(BOARD_INITPINS_LED_ORANGE_GPIO,
                     BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      g_Button1Press = false;
      break;
    case 2:
      strcpy(Status_String, "STATEF");
      Status_Index = 6.0;
      PRINTF("%1.1f[4]\r", Status_Index);
      PWM_OnOFF(0, 80);
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_RED_GPIO,
                     BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      g_Button1Press = false;
      break;
    default:
      break;
    }
  }
  SDK_ISR_EXIT_BARRIER;
}

void delay(void) {
  volatile uint32_t i = 0;
  for (i = 0; i < 50U; ++i) {
    __asm("NOP"); /* delay */
  }
}

/*!
 * @brief Init for CRC-16-CCITT.
 * @details Init CRC peripheral module for CRC-16/CCITT-FALSE protocol:
 *          width=16 poly=0x1021 init=0xffff refin=false refout=false
 * xorout=0x0000 check=0x29b1 http://reveng.sourceforge.net/crc-catalogue/
 * name="CRC-16/CCITT-FALSE"
 */
static void InitCrc16_CcittFalse(CRC_Type *base, uint32_t seed) {
  crc_config_t config;

  /*
   * config.polynomial = 0x1021;
   * config.seed = 0xFFFF;
   * config.reflectIn = false;
   * config.reflectOut = false;
   * config.complementChecksum = false;
   * config.crcBits = kCrcBits16;
   * config.crcResult = kCrcFinalChecksum;
   */
  CRC_GetDefaultConfig(&config);
  config.seed = seed;
  CRC_Init(base, &config);
}

uint32_t GetCRC(uint32_t *data, uint32_t size) {
  /* ***************
   * CRC-16/CCITT-FALSE *
   *************** */
  uint32_t checksum16 = 0;
  InitCrc16_CcittFalse(base, 0xFFFFU);
  CRC_WriteData(base, (uint8_t *)data, size);
  checksum16 = CRC_Get16bitResult(base);

  return checksum16;
}

void PrepareSPIPacket(void) {
  /* junk response of 4B of command */
  uint32_t dummy = 0xaabbccddU;
  sendBuff[1] = (uint8_t)(dummy >> 24);
  sendBuff[0] = (uint8_t)(dummy >> 16);
  sendBuff[3] = (uint8_t)(dummy >> 8);
  sendBuff[2] = (uint8_t)(dummy >> 0);

  sendBuff[5] = (uint8_t)(ui_buff[0] >> 24);
  sendBuff[4] = (uint8_t)(ui_buff[0] >> 16);
  sendBuff[7] = (uint8_t)(ui_buff[0] >> 8);
  sendBuff[6] = (uint8_t)(ui_buff[0] >> 0);

  sendBuff[9] = (uint8_t)(ui_buff[2] >> 24);
  sendBuff[8] = (uint8_t)(ui_buff[2] >> 16);
  sendBuff[11] = (uint8_t)(ui_buff[2] >> 8);
  sendBuff[10] = (uint8_t)(ui_buff[2] >> 0);

  sendBuff[13] = (uint8_t)(ui_buff[4] >> 24);
  sendBuff[12] = (uint8_t)(ui_buff[4] >> 16);
  sendBuff[15] = (uint8_t)(ui_buff[4] >> 8);
  sendBuff[14] = (uint8_t)(ui_buff[4] >> 0);

  sendBuff[17] = (uint8_t)(ui_buff[1] >> 24);
  sendBuff[16] = (uint8_t)(ui_buff[1] >> 16);
  sendBuff[19] = (uint8_t)(ui_buff[1] >> 8);
  sendBuff[18] = (uint8_t)(ui_buff[1] >> 0);

  sendBuff[21] = (uint8_t)(ui_buff[3] >> 24);
  sendBuff[20] = (uint8_t)(ui_buff[3] >> 16);
  sendBuff[23] = (uint8_t)(ui_buff[3] >> 8);
  sendBuff[22] = (uint8_t)(ui_buff[3] >> 0);

  sendBuff[25] = (uint8_t)(ui_buff[5] >> 24);
  sendBuff[24] = (uint8_t)(ui_buff[5] >> 16);
  sendBuff[27] = (uint8_t)(ui_buff[5] >> 8);
  sendBuff[26] = (uint8_t)(ui_buff[5] >> 0);

  /* filling 4Bytes CRC value at the end */
  uint32_t data[] = {ui_buff[0], ui_buff[2], ui_buff[4],
                     ui_buff[1], ui_buff[3], ui_buff[5]};
  uint32_t crc_val = GetCRC(data, sizeof(data));
  sendBuff[29] = crc_val >> 24;
  sendBuff[28] = crc_val >> 16;
  sendBuff[31] = crc_val >> 8;
  sendBuff[30] = crc_val >> 0;
}

/*
 * @brief This is the interrupt callback function for SAR ADC module.
 */
void SARADCCallback(void) {
  spi_transfer_t xfer = {0};

  if (adc_hwtrigger_counter < (1U * SAMPLE_SIZE * 50U)) {
    /* about 1 seconds check */
    adc_hwtrigger_counter++;
    if (adc_hwtrigger_counter == 1 * SAMPLE_SIZE * 50) {
      adc_hwtrigger_detected = true;
    }
  }

  if ((ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
    ui_buff[0] = (uint16)ADC16_GetChannelConversionValue(ADC0, 0U);
    /* read the SD-AFE current channel sample */
    ui_buff[1] = AFE->RR[0];

    /* Replace ui_buff[] with sine-table */
    ui_buff[0] = (uint32_t)sin_64s_4e6_5h_40p_sh_offs[sample_index];
    ui_buff[1] = (uint32_t)current_sense_buf[0][sample_index];
  }
  if ((ADC0->SC1[1] & ADC_SC1_COCO_MASK)) {
    ui_buff[2] = (uint16)ADC16_GetChannelConversionValue(ADC0, 1U);
    /* read the SD-AFE current channel sample */
    ui_buff[3] = AFE->RR[1];

    /* Replace ui_buff[] with sine-table */
    ui_buff[2] = (uint32_t)sin_64s_4e6_5h_40p_sh_offs[sample_index];
    ui_buff[3] = (uint32_t)current_sense_buf[1][sample_index];
  }
  if ((ADC0->SC1[3] & ADC_SC1_COCO_MASK)) {
    tmp16 = (uint16)ADC16_GetChannelConversionValue(ADC0, 3U);

    dummy = AFE->RR[3];
    read_pot_val = true;
  }
  if ((ADC0->SC1[2] & ADC_SC1_COCO_MASK) &&
      (ADC0->SC1[2] & ADC_SC1_AIEN_MASK)) {

    ui_buff[4] = (uint16)ADC16_GetChannelConversionValue(ADC0, 2U);
    /* read the SD-AFE current channel sample */
    ui_buff[5] = AFE->RR[2];

    /* Replace ui_buff[] with sine-table */
    ui_buff[4] = (uint32_t)sin_64s_4e6_5h_40p_sh_offs[sample_index];
    ui_buff[5] = (uint32_t)current_sense_buf[2][sample_index];

    sample_index++;
    if (sample_index == SAMPLE_SIZE) {
      sample_index = 0;
    }

    /* ui_buff got uint8 24 bytes, send it now */
    last_transfer_done = false;

    PrepareSPIPacket();
    xfer.txData = sendBuff;
    xfer.rxData = buff;
    xfer.dataSize = BUFFER_SIZE;

    SPI_SlaveTransferDMA(SPI_SLAVE, &s_handle, &xfer);

    /* toggle DRDY signal */
    GPIO_PortToggle(GPIOL, 1u << 2);
    delay();
    GPIO_PortToggle(GPIOL, 1u << 2);
  }

  SDK_ISR_EXIT_BARRIER;
}

/*
 * @brief This handler is used to set a LCD pin segment.
 */
void SLCD_SetLCDPin(lcd_set_type_t type, uint32_t lcd_pin, uint8_t pin_val,
                    int32_t on) {
  /* LCD segment pin and slcd gpio pin number map. */
  static uint8_t slcd_lcd_gpio_seg_pin[SLCD_PIN_NUM] = {
      38, 36, 34, 32, 31, 29, 25, 23, 43, 37,
      35, 33, 50, 30, 45, 24, 26, 28, 44, 59};
  assert(lcd_pin > 0);

  uint8_t gpio_pin = 0;
  uint8_t bit_val = 0;
  uint8_t i = 0;

  /* lcd _pin starts from 1. */
  gpio_pin = slcd_lcd_gpio_seg_pin[lcd_pin - 1U];

  if (type == SLCD_Set_Num) {
    SLCD_SetFrontPlaneSegments(LCD, gpio_pin, (on ? pin_val : 0));
  } else {
    for (i = 0; i < 8; ++i) {
      bit_val = (uint8_t)((pin_val >> i) & 0x1U);
      if (bit_val != 0) {
        SLCD_SetFrontPlaneOnePhase(LCD, gpio_pin, (slcd_phase_index_t)i, on);
      }
    }
  }
}

/*
 * @brief This handler is used to initialize the LCD section.
 */
void SLCD_APP_Init(void) {
  slcd_config_t config;
  slcd_clock_config_t clkConfig = {
      kSLCD_DefaultClk,
      kSLCD_AltClkDivFactor1,
      kSLCD_ClkPrescaler01,
  };

  /* Get Default configuration. */
  /*
   * config.displayMode = kSLCD_NormalMode;
   * config.powerSupply = kSLCD_InternalVll3UseChargePump;
   * config.voltageTrim = kSLCD_RegulatedVolatgeTrim00;
   * config.lowPowerBehavior = kSLCD_EnabledInWaitStop;
   * config.frameFreqIntEnable = false;
   * config.faultConfig = NULL;
   */
  SLCD_GetDefaultConfig(&config);

  /* Verify and Complete the configuration structure. */
  config.clkConfig = &clkConfig;
  config.loadAdjust = kSLCD_HighLoadOrSlowestClkSrc;
  config.dutyCycle = kSLCD_1Div8DutyCycle;
  /* LCD_P31/P30/P29/P28/P26/P25/P24/P23/P22/P20/P19/P14/P13. */
  config.slcdLowPinEnabled =
      (offLow32Pin(14) | offLow32Pin(20) | offLow32Pin(22) | offLow32Pin(13) |
       offLow32Pin(19) | offLow32Pin(28) | offLow32Pin(26) | offLow32Pin(24) |
       offLow32Pin(31) | offLow32Pin(29) | offLow32Pin(25) | offLow32Pin(23) |
       offLow32Pin(30));
  /* LCD_P59/P58/P57/P56/P50/P45/P44/P43/P38/P37/P36/P35/P34/P33/P32. */
  config.slcdHighPinEnabled =
      (offHigh32Pin(56) | offHigh32Pin(58) | offHigh32Pin(57) |
       offHigh32Pin(59) | offHigh32Pin(44) | offHigh32Pin(45) |
       offHigh32Pin(38) | offHigh32Pin(36) | offHigh32Pin(34) |
       offHigh32Pin(32) | offHigh32Pin(43) | offHigh32Pin(37) |
       offHigh32Pin(35) | offHigh32Pin(33) | offHigh32Pin(50));
  /* LCD_P22/20/19/14/13 --> b22/b20/b19/b14/b13 = 1. */
  config.backPlaneLowPin =
      (offLow32Pin(14) | offLow32Pin(20) | offLow32Pin(22) | offLow32Pin(13) |
       offLow32Pin(19));
  /* LCD_P58/57/56 --> b26/b25/b24 = 1. */
  config.backPlaneHighPin =
      (offHigh32Pin(56) | offHigh32Pin(58) | offHigh32Pin(57));
  SLCD_Init(LCD, &config);
}

/*
 * @brief This handler is used to clear the LCD screen.
 */
void SLCD_Clear_Screen(void) {
  /* Disables all front plane pins on all eight phases Phase A ~ Phase H.
  P59/P50/P45/P44/P43/P38/P37/P36/P35/P34/P33/P32/P31/P30/P29/P28/P26/P25/P24/P23
*/
  SLCD_SetFrontPlaneSegments(LCD, 23, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 24, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 25, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 26, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 28, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 29, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 30, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 31, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 32, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 33, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 34, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 35, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 36, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 37, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 38, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 43, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 44, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 45, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 50, (uint8_t)kSLCD_NoPhaseActivate);
  SLCD_SetFrontPlaneSegments(LCD, 59, (uint8_t)kSLCD_NoPhaseActivate);
}

/*
 * @brief This handler is used to reset the message on LCD screen.
 */
static void SLCD_Set_Display_Ready(tSLCD_Engine *slcd_engine) {
  uint32_t pinNum = 0;
  uint8_t allPhaseOn = (uint8_t)(kSLCD_PhaseAActivate | kSLCD_PhaseBActivate |
                                 kSLCD_PhaseCActivate | kSLCD_PhaseDActivate |
                                 kSLCD_PhaseEActivate | kSLCD_PhaseFActivate |
                                 kSLCD_PhaseGActivate | kSLCD_PhaseHActivate);

  for (pinNum = 0; pinNum < FSL_FEATURE_SLCD_HAS_PIN_NUM; pinNum++) {
    SLCD_SetFrontPlaneSegments(LCD, pinNum, allPhaseOn);
  }

  SLCD_SetBackPlanePhase(LCD, 14, kSLCD_PhaseHActivate);
  SLCD_SetBackPlanePhase(LCD, 20, kSLCD_PhaseGActivate);
  SLCD_SetBackPlanePhase(LCD, 22, kSLCD_PhaseFActivate);
  SLCD_SetBackPlanePhase(LCD, 56, kSLCD_PhaseEActivate);
  SLCD_SetBackPlanePhase(LCD, 58, kSLCD_PhaseDActivate);
  SLCD_SetBackPlanePhase(LCD, 13, kSLCD_PhaseCActivate);
  SLCD_SetBackPlanePhase(LCD, 19, kSLCD_PhaseBActivate);
  SLCD_SetBackPlanePhase(LCD, 57, kSLCD_PhaseAActivate);

  SLCD_StartDisplay(LCD);
  SLCD_Clear_Screen();
  SLCD_Engine_Show_Icon(slcd_engine, ICON_L1, 1);
  /* SLCD_Engine_Show_Icon(slcd_engine, ICON_RMS, 1); */
  SLCD_Engine_Show_Icon(slcd_engine, ICON_S16, 1);
  SLCD_Engine_Show_Icon(slcd_engine, ICON_S33, 1);
  SLCD_Engine_Show_Icon(slcd_engine, ICON_P3, 1);
}

/*
 * @brief This handler is used to simulate a sine wave and do metrology
 * function.
 */
static void Run_MeterLib(void) {
  static double U_ANGLE = (45.0 / 180.0) * PI;
  static double I_SHIFT = (-5.5 / 180.0) * PI;
  static tMETERLIB1PH_DATA mlib = METERLIB1PH_CFG;
  static double time = 0.0;
  static int cycle = 0;

  /* calculate phase voltage and phase current waveforms                    */
  time = time + (1.0 / KWH_CALC_FREQ);
  // simulated calculated voltage waveform
  u24_sample = (frac32)FRAC24(
      (((sin((2 * PI * 50.0 * time) + U_ANGLE) * 230.0 * sqrt(2)) + 0.0) /
       U_MAX));

  /* simulated calculated current waveform */
  /* Calculate current sample based on Pot reading */
  i24_sample = (frac32)FRAC24(
      ((sin((2 * PI * 50.0 * time) + I_SHIFT) * Pot_Value * sqrt(2) + 0.0) /
       I_MAX));

  METERLIB1PH_ProcSamples(&mlib, u24_sample, i24_sample, &shift);
  METERLIB1PH_CalcWattHours(&mlib, &wh_cnt, METERLIB_KWH_PR(IMP_PER_KWH));

  /* functions below might be called less frequently - please refer to      */
  /* KWH_CALC_FREQ, KVARH_CALC_FREQ and DECIM_FACTOR constants              */
  if (!(cycle % (int)(KWH_CALC_FREQ / KVARH_CALC_FREQ))) {
    METERLIB1PH_CalcVarHours(&mlib, &varh_cnt,
                             METERLIB_KVARH_PR(IMP_PER_KVARH));
  }

  if (!(cycle % DECIM_FACTOR)) {
    METERLIB1PH_CalcAuxiliary(&mlib);
  }

  METERLIB1PH_ReadResults(&mlib, &U_RMS, &I_RMS, &P, &Q, &S);

  cycle++;
}

void Update_current_sense_data(void) {
  static int sample_index = 0;

#if (SAMPLE_SIZE == 64)
  current_sense_buf[0][sample_index] = sin_64s_6e6_5h_10p[sample_index];
  current_sense_buf[1][sample_index] = sin_64s_6e6_5h_10p[sample_index];
  current_sense_buf[2][sample_index] = sin_64s_6e6_5h_10p[sample_index];
#endif
#if (SAMPLE_SIZE == 120)
  current_sense_buf[0][sample_index] = sin_120s_6e6_5h_10p[sample_index];
  current_sense_buf[1][sample_index] = sin_120s_6e6_5h_10p[sample_index];
  current_sense_buf[2][sample_index] = sin_120s_6e6_5h_10p[sample_index];
#endif

  current_sense_buf[0][sample_index] *= (int)Pot_Value;
  current_sense_buf[1][sample_index] *= (int)Pot_Value;
  current_sense_buf[2][sample_index] *= (int)Pot_Value;

  current_sense_buf[0][sample_index] /= I_MAX_BRD;
  current_sense_buf[1][sample_index] /= I_MAX_BRD;
  current_sense_buf[2][sample_index] /= I_MAX_BRD;

  sample_index++;
  sample_index %= SAMPLE_SIZE;
}

/*
 * Timer handler to sequence LED blink Rate and LCD refresh rate
 */
void LPTMR0_IRQHandler(void) {
  LPTMR_ClearStatusFlags(LPTMR0, (uint32_t)kLPTMR_TimerCompareFlag);

  if (read_pot_val) {
    uint32_t tmp32;
    uint8_t tmpnum;

    tmp32 = tmp32_2;
    tmpnum = (uint8_t)(tmp32 / 1000U);
    tmp32 = tmp32 % 1000U;
    SLCD_Engine_Show_Num(&slcd_engine, (int32_t)tmpnum, NUM_POS5, 1);

    tmpnum = (uint8_t)(tmp32 / 100U);
    tmp32 = tmp32 % 100U;
    SLCD_Engine_Show_Num(&slcd_engine, (int32_t)tmpnum, NUM_POS6, 1);

    tmpnum = (uint8_t)(tmp32 / 10U);
    tmp32 = tmp32 % 10U;
    SLCD_Engine_Show_Num(&slcd_engine, (int32_t)tmpnum, NUM_POS7, 1);

    tmpnum = (uint8_t)(tmp32);
    SLCD_Engine_Show_Num(&slcd_engine, (int32_t)tmpnum, NUM_POS8, 1);

    tmpnum = (uint8_t)(Status_Index / 1);
    SLCD_Engine_Show_Num(&slcd_engine, (int32_t)tmpnum, NUM_POS15, 1);

    if ((Error_Counter == 1U) || (Error_Counter == 2U)) {
      SLCD_Engine_Show_Icon(&slcd_engine, ICON_S29, 1);
    }
    SLCD_Engine_Show_Icon(&slcd_engine, ICON_L1, 1);
    read_pot_val = false;
  } else if ((afe_inactivite_counter < 5U) && (!adc_hwtrigger_detected)) {
    afe_inactivite_counter++;

    if (afe_inactivite_counter == 5U) {
      /* AFE clock isn't connected for 5 seconds, so need to re-init SAR-ADC for
       * software self-trigger */
      InitSARADC(kSWTriggerMode);

      /* this enables Systick_Handler to start software trigger and read ADC
       * value */
      afe_run_failed = true;
    } else {
    }
  }

  if (g_Button1Press) {
    /* Toggle LED. */
    switch (led_turn) {
    case 0:
      GPIO_PortClear(BOARD_INITPINS_LED_ORANGE_GPIO,
                     BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      break;
    case 1:
      GPIO_PortClear(BOARD_INITPINS_LED_ORANGE_GPIO,
                     BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_RED_GPIO,
                     BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      break;
    case 2:
      GPIO_PortClear(BOARD_INITPINS_LED_ORANGE_GPIO,
                     BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_RED_GPIO,
                     BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_GREEN_GPIO,
                     BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      break;
    case 3:
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      break;
    }
    led_turn++;
    if (led_turn > 3U) {
      led_turn = 0;
    }
  }
}

/*
 * LPTMR is used to create refresh timing for the onboard LCD
 */
static void Start_Timer(void) {
  lptmr_config_t lptmrConfig;
  /* Configure LPTMR */
  /*
   * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
   * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
   * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
   * lptmrConfig.enableFreeRunning = false;
   * lptmrConfig.bypassPrescaler = true;
   * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
   * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
   */
  LPTMR_GetDefaultConfig(&lptmrConfig);
  /* Initialise the lptmr */
  LPTMR_Init(LPTMR0, &lptmrConfig);

  /* Set timer period */
  /* set LPTMR to 1 Second interval */
  LPTMR_SetTimerPeriod(LPTMR0,
                       (uint32_t)(USEC_TO_COUNT(750000u, LPTMR_CLK_FREQ)));
  /* Enable timer interrupt */
  LPTMR_EnableInterrupts(LPTMR0, (uint32_t)kLPTMR_TimerInterruptEnable);

  /* Enable at the NVIC */
  // EnableIRQ(LPTMR0_LPTMR1_IRQn);
  (void)EnableIRQWithPriority(LPTMR0_LPTMR1_IRQn, 2U);
  LPTMR_StartTimer(LPTMR0);
}

/*
 * Monitor UART RX for valid commands from Host
 */
static void Process_HostCommand(void) {
  char ch_in;
  uint8_t loop;

  loop = 1;
  while (loop) {
    ch_in = (char)GETCHAR();

    if ((ch_in < '0') || (ch_in > 'F')) {
      /* command received is invalid, restart while loop immediately*/
      continue;
    }

    switch (ch_in) {
    case '0':
      PRINTF("%3.2f[1]%3.2f[2]%4.2f[3]%1.1f[4]\r", I_RMS, U_RMS, P,
             Status_Index);
      loop = 0;
      break;
    case '1':
      PRINTF("%3.2f[1]\r", I_RMS);
      loop = 0;
      break;
    case '2':
      PRINTF("%3.2f[2]\r", U_RMS);
      loop = 0;
      break;
    case '3':
      PRINTF("%4.2f[3]\r", P);
      loop = 0;
      break;
    case '4':
      PRINTF("%1.1f[4]\r", Status_Index);
      loop = 0;
      break;
    case '5':
      PRINTF("%s[5]\r", TWRKM35Z75M_VERSION);
      loop = 0;
      break;
    case 'A':
      strcpy(Status_String, "STATEA");
      Status_Index = 1.0;
      PWM_OnOFF(0, 53);
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_GREEN_GPIO,
                   BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      State_Counter = 1;
      Error_Counter = 0;
      g_Button1Press = false;
      loop = 0;
      break;
    case 'B':
      Status_Index = 2.0; // STATEB
      strcpy(Status_String, "STATEB");
      PWM_OnOFF(0, 53);
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_GREEN_GPIO,
                     BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      State_Counter = 2;
      g_Button1Press = false;
      loop = 0;
      break;
    case 'C':
      Status_Index = 3.0; // STATEC
      strcpy(Status_String, "STATEC");
      PWM_OnOFF(1, 53);
      /* Change state of button. */
      GPIO_PortSet(BOARD_INITPINS_LED_ORANGE_GPIO,
                   BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      led_turn = 0;
      g_Button1Press = true;
      State_Counter = 3;
      loop = 0;
      break;
    case 'D':
      Status_Index = 4.0; // STATED
      strcpy(Status_String, "STATED");
      PWM_OnOFF(0, 53);
      /* Change state of button. */
      GPIO_PortClear(BOARD_INITPINS_LED_ORANGE_GPIO,
                     BOARD_INITPINS_LED_ORANGE_GPIO_PIN_MASK);
      GPIO_PortSet(BOARD_INITPINS_LED_RED_GPIO,
                   BOARD_INITPINS_LED_RED_GPIO_PIN_MASK);
      GPIO_PortClear(BOARD_INITPINS_LED_GREEN_GPIO,
                     BOARD_INITPINS_LED_GREEN_GPIO_PIN_MASK);
      g_Button1Press = false;
      State_Counter = 4;
      loop = 0;
      break;
    case 'E':
      Status_Index = 5.0; // STATEE
      loop = 0;
      break;
    case 'F':
      Status_Index = 6.0; // STATEF
      loop = 0;
      break;
    default:
      /*Execute these statements when the result of expression Not matching with
       * any Option */
      break;
    }
  }
}

void PWM_OnOFF(uint8_t onoff, uint8_t perc) {
  qtmr_config_t qtmrConfig;
  /*
   * qtmrConfig.debugMode = kQTMR_RunNormalInDebug;
   * qtmrConfig.enableExternalForce = false;
   * qtmrConfig.enableMasterMode = false;
   * qtmrConfig.faultFilterCount = 0;
   * qtmrConfig.faultFilterPeriod = 0;
   * qtmrConfig.primarySource = kQTMR_ClockDivide_2;
   * qtmrConfig.secondarySource = kQTMR_Counter0InputPin;
   */

  if ((onoff == 1) && (PWMRunning == false)) {
    QTMR_GetDefaultConfig(&qtmrConfig);
    /* Use IP bus clock div by 8 */
    qtmrConfig.primarySource = kQTMR_ClockDivide_8;
    QTMR_Init(TMR1, &qtmrConfig);
    /* Generate a 1Khz PWM signal with 70% high duty cycle */
    QTMR_SetupPwm(TMR1, 1000U, perc, false, BUS_CLK_FREQ / 8U);
    /* Start the counter */
    QTMR_StartTimer(TMR1, kQTMR_PriSrcRiseEdge);
    PWMRunning = true;
  }
  if ((onoff == 0) && (PWMRunning == true)) {
    QTMR_StopTimer(TMR1);
    PWMRunning = false;
  }
}

/*
 * Handler routine used to obtain set value of R21 potentiometer, scale to max
 * current and then use this value when running the metrology library This
 * occurs every 10mS
 */
void SysTick_Handler(void) {
  float RMS_Voltage; // calculated voltage

  if (afe_run_failed) {
    /* Obtain latest ADC value */
    while (0U == ((uint32_t)kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(DEMO_ADC16_BASEADDR,
                                              DEMO_ADC16_CHANNEL_GROUP))) {
    }
    tmp16 = (uint16_t)ADC16_GetChannelConversionValue(DEMO_ADC16_BASEADDR,
                                                      DEMO_ADC16_CHANNEL_GROUP);

    /* start another conversion */
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP,
                           &adc16ChannelConfigStruct);

    read_pot_val = true;
  }
  /* scaling amps to max 32 */
  tmp32_2 =
      (3200U * tmp16) / 65535U; /* 65535 = 2^16 - 1 (16 bit ADC max. value) */

  Pot_Value = (((float)tmp16) / 65535) * I_MAX_BRD;

  RMS_Voltage = Pot_Value * 10.0;

  Run_MeterLib();
  Update_current_sense_data();
}

void ConnectAFEtoSARADC(void) {
  /* Configure the XBAR signal connections. */
  XBAR_SetSignalsConnection(XBAR, kXBAR_InputAfeCh0CocFlag,
                            kXBAR_OutputAdcTrgA);
  XBAR_SetSignalsConnection(XBAR, kXBAR_InputAfeCh1CocFlag,
                            kXBAR_OutputAdcTrgB);
  XBAR_SetSignalsConnection(XBAR, kXBAR_InputAfeCh2CocFlag,
                            kXBAR_OutputAdcTrgC);
  XBAR_SetSignalsConnection(XBAR, kXBAR_InputAfeCh3CocFlag,
                            kXBAR_OutputAdcTrgD);
}
/*
 * @brief This handler is used to initialize SARADC, 4 channels of SAR ADC have
 * been enabled with one of them generating interrupts.
 */
void InitSARADC(adc_trig_mode_t trigger_mode) {
  adc16_config_t adc16ConfigStruct;

  /* de-init if already initialized */
  ADC16_Deinit(ADC0);

  /*
   * Initialization ADC for
   * 16bit resolution, interrupt mode, hw trigger enabled.
   * normal convert speed, VREFH/L as reference,
   * disable continuous convert mode.
   */
  /*
   * adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
   * adcUserConfig.clockSource = kADC16_ClockSourceAsynchronousClock;
   * adcUserConfig.enableAsynchronousClock = true;
   * adcUserConfig.clockDivider = kADC16_ClockDivider8;
   * adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
   * adcUserConfig.longSampleMode = kADC16_LongSampleDisabled;
   * adcUserConfig.enableHighSpeed = false;
   * adcUserConfig.enableLowPower = false;
   * adcUserConfig.enableContinuousConversion = false;
   */
  ADC16_GetDefaultConfig(&adc16ConfigStruct);

  if (trigger_mode == kHWTriggerMode) {
    adc16ConfigStruct.referenceVoltageSource =
        kADC16_ReferenceVoltageSourceVref;
    adc16ConfigStruct.clockSource = kADC16_ClockSourceAlt1;
    adc16ConfigStruct.enableAsynchronousClock = false;
    adc16ConfigStruct.clockDivider = kADC16_ClockDivider4;
    adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;
    adc16ConfigStruct.enableHighSpeed = true;
    ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);

    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, true);
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    /* Configure channel A : emulation a Voltage sense channel */
    adc16ChannelConfigStruct.channelNumber = 0U;
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, 0U, &adc16ChannelConfigStruct);
    /* Configure channel B : emulation a Voltage sense channel  */
    adc16ChannelConfigStruct.channelNumber = 1U;
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, 1U, &adc16ChannelConfigStruct);
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;
    /* Configure channel C : emulation a Voltage sense channel  */
    adc16ChannelConfigStruct.channelNumber = 2U;
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, 2U, &adc16ChannelConfigStruct);
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    /* Configure channel D : Start Potentiometer */
    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, 3U, &adc16ChannelConfigStruct);

    ADC16_EnableDMA(DEMO_ADC16_BASEADDR, false);
    NVIC_SetPriority(DEMO_ADC16_IRQN, 1U);
    NVIC_EnableIRQ(DEMO_ADC16_IRQN);
  } else {
    NVIC_DisableIRQ(DEMO_ADC16_IRQN);

    /* Measure R21 value connected to the 16 bit ADC channel 8 */
    /* Start Potentiometer */

    adc16ConfigStruct.clockDivider = kADC16_ClockDivider2;
    adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;
    ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(
        DEMO_ADC16_BASEADDR,
        false); /* Make sure the software trigger is used. */
    ADC16_SetHardwareAverage(DEMO_ADC16_BASEADDR, kADC16_HardwareAverageCount8);

    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    /*
         When in software trigger mode, each conversion is launched once calling
       the "ADC16_SetChannelConfig()" function, which works like writing a
       conversion command and executing it. For another channel's conversion,
       just to change the "channelNumber" field in channel's configuration
         structure, and call the ADC16_SetChannelConf() again.
     */
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP,
                           &adc16ChannelConfigStruct);

    while (0U == ((uint32_t)kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(DEMO_ADC16_BASEADDR,
                                              DEMO_ADC16_CHANNEL_GROUP))) {
    }
  }
}

void InitVREF(void) {
  uint32_t temp32;
  /* Do necessary initialization in the SIM module */
  temp32 = SIM->MISC_CTL &
           ~(SIM_MISC_CTL_VREFBUFPD_MASK | SIM_MISC_CTL_VREFBUFINSEL_MASK |
             SIM_MISC_CTL_VREFBUFOUTEN_MASK);
  temp32 |= SIM_MISC_CTL_VREFBUFPD(0) | SIM_MISC_CTL_VREFBUFINSEL(0) |
            SIM_MISC_CTL_VREFBUFOUTEN(1);
  SIM->MISC_CTL = temp32;

  /* VREF module must be initialized after SIM module                         */
  vref_config_t config;

  /* Get vref default configure */
  VREF_GetDefaultConfig(&config);
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) &&                             \
    FSL_FEATURE_VREF_HAS_LOW_REFERENCE
  /* Enable low reference volt */
  config.enableLowRef = true;
#endif /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
  /* Initialize vref */
  VREF_Init(VREF, &config);
}

void InitAFE(void) {
  uint8 i = 0;

  afe_config_t afeConfig;
  afe_channel_config_t afeChnConfig;

  afeConfig.enableLowPower = true;
  afeConfig.resultFormat = kAFE_ResultFormatRight;
  afeConfig.clockSource = kAFE_ClockSource3;
  afeConfig.clockDivider = kAFE_ClockDivider4;
  afeConfig.startupCount = 15U; /* startupCnt = (Clk_freq/Clk_div)*20e-6 */

  AFE_Init(AFE, &afeConfig);

  for (i = 0; i < 0xffU; i++) {
    ; /* add little start-up delay */
  }

  afeChnConfig.enableHardwareTrigger = false;
  afeChnConfig.enableContinuousConversion = true;
  afeChnConfig.channelMode = kAFE_BypassDisable;
  afeChnConfig.decimatorOversampleRatio = kAFE_DecimatorOversampleRatio512;
  afeChnConfig.pgaGainSelect = kAFE_PgaDisable;
  /* Initialize AFE to emulate to measure a Phase current */
  AFE_SetChannelConfig(AFE, 0U, &afeChnConfig);
  AFE_SetChannelPhaseDelayValue(AFE, 0U, 1U);
  /* Initialize AFE to emulate to measure a Phase current */
  AFE_SetChannelConfig(AFE, 1U, &afeChnConfig);
  AFE_SetChannelPhaseDelayValue(AFE, 1U, 60U);
  /* Initialize AFE to emulate to measure a Phase current */
  AFE_SetChannelConfig(AFE, 2U, &afeChnConfig);
  AFE_SetChannelPhaseDelayValue(AFE, 2U, 120U);
  /* Initialize AFE to emulate to measure neutral current */
  AFE_SetChannelConfig(AFE, 3U, &afeChnConfig);
  AFE_SetChannelPhaseDelayValue(AFE, 3U, 30U);
}

void MeteringInit(void) {
  mcg_pll_config_t pllConfig;
  pllConfig.refSrc = kMCG_PllRefRtc;
  pllConfig.enableMode = 0U;
  CLOCK_EnablePll0(&pllConfig);

  XBAR_Init(XBAR);

  ConnectAFEtoSARADC();
  InitSARADC(kHWTriggerMode);
  InitVREF();

  /* Now trigger the AFE channels after configuration */
  InitAFE();
  AFE_DoSoftwareTriggerChannel(AFE,
                               (AFE_CR_SOFT_TRG0_MASK | AFE_CR_SOFT_TRG1_MASK |
                                AFE_CR_SOFT_TRG2_MASK | AFE_CR_SOFT_TRG3_MASK));
}

/*
 * @brief SPI DMA Callback
 */
static void SPI_Slave_Callback(SPI_Type *base, spi_dma_handle_t *handle,
                               status_t status, void *userData) {
  slaveFinished = true;
  last_transfer_done = true;
}

void SPI_DMA_init(void) {
  spi_slave_config_t userConfig;

  /* Init DMAMUX */
#if FSL_FEATURE_DMA_MODULE_CHANNEL != FSL_FEATURE_DMAMUX_MODULE_CHANNEL
  DMAMUX_Init(EXAMPLE_TX_DMAMUX);
  DMAMUX_Init(EXAMPLE_RX_DMAMUX);
  DMAMUX_SetSource(EXAMPLE_TX_DMAMUX, EXAMPLE_SPI_TX_DMAMUX_CHANNEL,
                   SPI_TX_SOURCE);
  DMAMUX_SetSource(EXAMPLE_RX_DMAMUX, EXAMPLE_SPI_RX_DMAMUX_CHANNEL,
                   SPI_RX_SOURCE);
  DMAMUX_EnableChannel(EXAMPLE_TX_DMAMUX, EXAMPLE_SPI_TX_DMAMUX_CHANNEL);
  DMAMUX_EnableChannel(EXAMPLE_RX_DMAMUX, EXAMPLE_SPI_RX_DMAMUX_CHANNEL);

  /* Init the DMA module */
  DMA_Init(DMA);
  DMA_CreateHandle(&txHandle, DMA, EXAMPLE_SPI_TX_DMA_CHANNEL);
  DMA_CreateHandle(&rxHandle, DMA, EXAMPLE_SPI_RX_DMA_CHANNEL);
#else
  DMAMUX_Init(DMAMUX);
  DMAMUX_SetSource(DMAMUX, SPI_TX_CHANNEL, SPI_TX_SOURCE);
  DMAMUX_SetSource(DMAMUX, SPI_RX_CHANNEL, SPI_RX_SOURCE);
  DMAMUX_EnableChannel(DMAMUX, SPI_TX_CHANNEL);
  DMAMUX_EnableChannel(DMAMUX, SPI_RX_CHANNEL);

  /* Init the DMA module */
  DMA_Init(DMA);
  DMA_CreateHandle(&txHandle, DMA, SPI_TX_CHANNEL);
  DMA_CreateHandle(&rxHandle, DMA, SPI_RX_CHANNEL);
#endif

  SPI_SlaveGetDefaultConfig(&userConfig);
  userConfig.dataMode = kSPI_16BitMode;
  SPI_SlaveInit(SPI_SLAVE, &userConfig);
  /* This function registers callback for DMA channels and disable SPI-Fifo */
  SPI_SlaveTransferCreateHandleDMA(SPI_SLAVE, &s_handle, SPI_Slave_Callback,
                                   NULL, &txHandle, &rxHandle);

  Initialize_Sample_Buffer();

  /*
   * Set Data ready high
   */
  gpio_pin_config_t Data_RDY_config = {.pinDirection = kGPIO_DigitalOutput,
                                       .outputLogic = 1U};
  GPIO_PinInit(GPIOL, 2, &Data_RDY_config);
  GPIO_PortSet(GPIOL, 2);
}
/*!
 * @brief main function
 */
int main(void) {
  /* Init hardware. */
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  /* SLCD Initialization. */
  SLCD_APP_Init();

  memset(&slcd_engine, 0, sizeof(tSLCD_Engine));
  SLCD_Engine_Init(&slcd_engine, SLCD_SetLCDPin);

  SLCD_Set_Display_Ready(&slcd_engine);

  Start_Timer();

  /* Initialize SPI, DMA, Data Ready Pin */
  SPI_DMA_init();

  /* Initialize all metering specific MCU IPs */
  MeteringInit();

  /* Load Interrupt for board Switches. */
  EnableIRQ(BOARD_SW_IRQ);

  /* Set systick reload value to generate 10ms interrupt */
  if (SysTick_Config(SystemCoreClock / 100U)) {
    while (1) {
      /* arrives here only if Systick could not be setup correctly */
    }
  }

  while (1) {
    Process_HostCommand();
  }
}

void Initialize_Sample_Buffer(void) {
  for (uint16_t i = 0; i < SAMPLE_SIZE; i++) {
#if (SAMPLE_SIZE == 64)
    current_sense_buf[0][i] = sin_64s_6e6_5h_10p[i];
    current_sense_buf[1][i] = sin_64s_6e6_5h_10p[i];
    current_sense_buf[2][i] = sin_64s_6e6_5h_10p[i];
#endif
#if (SAMPLE_SIZE == 120)
    current_sense_buf[0][i] = sin_120s_6e6_5h_10p[i];
    current_sense_buf[1][i] = sin_120s_6e6_5h_10p[i];
    current_sense_buf[2][i] = sin_120s_6e6_5h_10p[i];
#endif
  }
}

/******************************************************************************
 *
 * Copyright 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 **************************************************************************************
 * Filter-Based Metering Library Configuration File, Created: Sun May 31 09:38:33 2015
 **************************************************************************************
 * @TAGNAME       = METERLIB1PH_CFG
 * @LOCKED        =         0
 * @FSAMPLE       =      1200
 * @DFACTOR       =         2
 * @IMAX          =  141.4214
 * @UMAX          =  350.0000
 * @FREQ          =        50
 * @COUNTERS_RES  =     10000
 * @PWR_THRESHOLD =    0.1000
 * @I_STARTING    =    0.0200
 * @APWR_OFS      =    0.0000
 * @RPWR_OFS      =    0.0000
 * @ENERGY_ATT    =    0.0000
 * @IMP_PER_KWH   =     50000
 * @IMP_PER_KVARH =     50000
 * @HPF_FCUT      =    0.3000
 * @LPF1_FCUT     =    0.5000
 * @LPF2_FCUT     =    3.0000
 * @KWIN_BETA     =    6.0672
 * @KWIN_GAIN     =    1.0000
 * @FIR_TAPS_CHG  =         0
 * @FIR_FREQ_MOD  =         0
 * @CUR_SENSOR    =         1
 * @LIB_TYPE      =         1
 * @MATH_TYPE     =         1
 * @KWH_ONLY      =         0
 * @SW_PH_CORR    =         0
 * @MCU_CORE      =         1
 **************************************************************************************/
#ifndef __METERLIB1PH_CFG_H
#define __METERLIB1PH_CFG_H

/**************************************************************************************
 * General parameters and scaling coefficients
 **************************************************************************************/
#define POWER_METER               1PH  /*!< Power meter topology                      */
#define LIBRARY_PREFIX       METERLIB  /*!< Library prefix; high-performance library  */
#define CURRENT_SENSOR   PROPORTIONAL  /*!< Current sensor output characteristic      */
#define I_MAX                 141.421  /*!< Maximal current I-peak in amperes         */
#define U_MAX                 350.000  /*!< Maximal voltage U-peak in volts           */
#define F_NOM                      50  /*!< Nominal frequency in Hz                   */
#define COUNTER_RES             10000  /*!< Resolution of energy counters in inc/kWh  */
#define IMP_PER_KWH             50000  /*!< Impulses per kWh                          */
#define IMP_PER_KVARH           50000  /*!< Impulses per kVARh                        */
#define DECIM_FACTOR                2  /*!< Auxiliary calculations decimation factor  */
#define KWH_CALC_FREQ        1200.000  /*!< Sample frequency in Hz                    */
#define KVARH_CALC_FREQ      1200.000  /*!< Sample frequency in Hz                    */
/**************************************************************************************
 * Filter-based metering algorithm configuration structure
 **************************************************************************************/
#define METERLIB1PH_CFG                                                               \
{                                                                                     \
  U_MAX,                                                                              \
  I_MAX,                                                                              \
  FRAC32(((+0.1000)/(U_MAX*I_MAX/2.0))),                                              \
  FRAC32((+0.0200)/I_MAX),                                                            \
  1,                                                                                  \
  {{0L,0L,0L},{0L,0L,0L}},                                                            \
  {{FRAC32(+0.99921521804155),FRAC32(-0.99921521804155),FRAC32(-0.99843043608309)}},  \
  {{FRAC32(+0.13165249758740),FRAC32(+0.13165249758740),FRAC32(-1.0)}},               \
  {{0L,0LL},{0L,0LL}},                                                                \
  {0L,0LL},                                                                           \
  {{0L,0LL},{0L,0LL}},                                                                \
  {  49,                                                                              \
    {                                                                                 \
      FRAC32(0.0),FRAC32(-0.00073728465714),FRAC32(0.0),FRAC32(-0.00196750272687),    \
      FRAC32(0.0),FRAC32(-0.00411945802255),FRAC32(0.0),FRAC32(-0.00756839142185),    \
      FRAC32(0.0),FRAC32(-0.01278720365088),FRAC32(0.0),FRAC32(-0.02040684105768),    \
      FRAC32(0.0),FRAC32(-0.03136483560542),FRAC32(0.0),FRAC32(-0.04728105184137),    \
      FRAC32(0.0),FRAC32(-0.07151114503989),FRAC32(0.0),FRAC32(-0.11276139617420),    \
      FRAC32(0.0),FRAC32(-0.20318408017719),FRAC32(0.0),FRAC32(-0.63356345988777),    \
      FRAC32(0.0),FRAC32(+0.63356345988777),FRAC32(0.0),FRAC32(+0.20318408017719),    \
      FRAC32(0.0),FRAC32(+0.11276139617420),FRAC32(0.0),FRAC32(+0.07151114503989),    \
      FRAC32(0.0),FRAC32(+0.04728105184137),FRAC32(0.0),FRAC32(+0.03136483560542),    \
      FRAC32(0.0),FRAC32(+0.02040684105768),FRAC32(0.0),FRAC32(+0.01278720365088),    \
      FRAC32(0.0),FRAC32(+0.00756839142185),FRAC32(0.0),FRAC32(+0.00411945802255),    \
      FRAC32(0.0),FRAC32(+0.00196750272687),FRAC32(0.0),FRAC32(+0.00073728465714),    \
      FRAC32(0.0)                                                                     \
    },                                                                                \
     25,                                                                              \
    {                                                                                 \
      FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),        \
      FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),        \
      FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),        \
      FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),FRAC16(0.0),        \
      FRAC16(-1.0)                                                                    \
    }                                                                                 \
  },                                                                                  \
  {                                                                                   \
    {                                                                                 \
      0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,  \
      0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L            \
    },                                                                                \
    0LL,                                                                              \
    {                                                                                 \
      0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L,0L      \
    },                                                                                \
    0L                                                                                \
  },                                                                                  \
  {                                                                                   \
    {FRAC32(+0.00261116383261),FRAC32(+0.00261116383261),FRAC32(-0.99477767233478)},  \
    {FRAC32(+0.00261116383261),FRAC32(+0.00261116383261),FRAC32(-0.99477767233478)},  \
  },                                                                                  \
  {0LL,0LL,0L,0LL},                                                                   \
  {0LL,0LL,0L,0LL},                                                                   \
  {0LL,0LL,0L,0LL},                                                                   \
  {0LL,0LL,0L,0LL},                                                                   \
  {                                                                                   \
    FRAC48((+0.0000/(U_MAX*I_MAX))),FRAC32(+1.0000),  METERLIB_KWH_DR(   10000),      \
    {0LL,0LL,0LL},0LL,0L,FRAC16(-1.0),                                                \
    {FRAC32(+0.00779293629195),FRAC32(+0.00779293629195),FRAC32(-0.98441412741610)},  \
    {0LL,0LL,0LL},{0LL,0LL,0LL}                                                       \
  },                                                                                  \
  {                                                                                   \
    FRAC48((+0.0000/(U_MAX*I_MAX))),FRAC32(+1.0000),METERLIB_KVARH_DR(   10000),      \
    {0LL,01L,01L},0LL,0L,FRAC16(-1.0),                                                \
    {FRAC32(+0.00779293629195),FRAC32(+0.00779293629195),FRAC32(-0.98441412741610)},  \
    {0LL,0LL,0LL},{0LL,0LL,0LL}                                                       \
  }                                                                                   \
}
#endif  /* __METERLIB1PH_CFG_H */

/*
 * Copyright 2019, 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "app.h"
#include "clock_config.h"
#include "pin_mux.h"

#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_xbar.h"
#include "fsl_adc16.h"
#include "fsl_lptmr.h"
#include "fsl_vref.h"
#include "fsl_afe.h"
#include "fsl_spi_dma.h"
#include "fsl_dmamux.h"
#include "fsl_crc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_IRQN		ADC0_IRQn

#define DEMO_ADC16_BASEADDR ADC0
#define DEMO_ADC16_IRQN		ADC0_IRQn
/* Low Power Timer */
#define LPTMR_CLK_FREQ CLOCK_GetFreq(kCLOCK_LpoClk)
#define LPTMR0_IRQHandler LPTMR0_LPTMR1_IRQHandler

#define SARADCCallback ADC0_IRQHandler

#define SPI_SLAVE        SPI0
#define DMA              DMA0
/* #define DMAMUX           DMAMUX */
#define SPI_TX_CHANNEL   3U
#define SPI_RX_CHANNEL   2U
#define SPI_TX_SOURCE    kDmaRequestMux0SPI0Tx
#define SPI_RX_SOURCE    kDmaRequestMux0SPI0Rx
#define SPI_SOURCE_CLOCK kCLOCK_CoreSysClk

#define SAMPLE_SIZE 64		/* Number of samples per sine wave cycle: 64 or 120 */
#define DUMP_SAMPLES_I1 1
//#define DUMP_SAMPLES_I2 1
//#define DUMP_SAMPLES_I3 1

#define MCU_CLK         71991296U     /* MCU Core  frequency                   */
#define BUS_DIV         2            /* BUS clock divider  frequency          */
#define AFE_CLK         3.690e6     /* AFE frequency                         */
#define SAR_CONT        12.000e-6    /* SAR conversion time 12us / ~9MHz      */
  /* SAR conversion time based on  MCU (BUS) CLK                              */
#define SAR_SCT         (int16_t) ((SAR_CONT * MCU_CLK) / (BUS_DIV * 4))
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
lptmr_config_t g_lptmrConfig;
bool read_adc_val = false;
bool spi_pin_init = false ;
volatile uint32_t g_systickCounter;
/* SPI-DMA variables */
uint32_t ui_buff[6];
uint32_t dummy;
uint16_t adc_count = 0;
#define BUFFER_SIZE (4 + 24 + 4 ) /*  4B command + 24B data + 4B CRC */
static uint8_t buff[BUFFER_SIZE];
static uint8_t sendBuff[BUFFER_SIZE] = {0};
static spi_dma_handle_t s_handle;
static dma_handle_t txHandle;
static dma_handle_t rxHandle;
static volatile bool slaveFinished = false;
spi_transfer_t xfer = {0};
bool last_transfer_done = true ;
CRC_Type *base = CRC0;
uint8_t afe_inactivite_counter = 0;
bool afe_clk_detected = false;
bool afe_run_failed = false;
uint16_t g_crc_val;

#if ((DUMP_SAMPLES_I1 | DUMP_SAMPLES_I2 | DUMP_SAMPLES_I3) == 1)
int32_t dump_ui_buff[2][64];	// dump buffer for one channel (V/I)
uint16_t dump_buff_index = 0;
uint16_t dump_buff_count = 0;
#endif

/*
 * Set Data ready high
 */
gpio_pin_config_t Data_RDY_config = {
		.pinDirection = kGPIO_DigitalOutput,
		.outputLogic = 1U
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * @brief SPI DMA Callback
 */
static void SPI_Slave_Callback(SPI_Type *base, spi_dma_handle_t *handle, status_t status, void *userData)
{
	last_transfer_done = true ;
}

/*!
 *@ SPI DMA Init
 */
void SPI_DMA_init()
{
	spi_slave_config_t userConfig;

	/* Init DMAMUX */
#if FSL_FEATURE_DMA_MODULE_CHANNEL != FSL_FEATURE_DMAMUX_MODULE_CHANNEL
	DMAMUX_Init(EXAMPLE_TX_DMAMUX);
	DMAMUX_Init(EXAMPLE_RX_DMAMUX);
	DMAMUX_SetSource(EXAMPLE_TX_DMAMUX, EXAMPLE_SPI_TX_DMAMUX_CHANNEL, SPI_TX_SOURCE);
	DMAMUX_SetSource(EXAMPLE_RX_DMAMUX, EXAMPLE_SPI_RX_DMAMUX_CHANNEL, SPI_RX_SOURCE);
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
	userConfig.dataMode = kSPI_16BitMode ;
	userConfig.txWatermark = kSPI_TxFifoOneFourthEmpty;
	SPI_SlaveInit(SPI_SLAVE, &userConfig);
	/* This function registers callback for DMA channels and disable SPI-Fifo */
	SPI_SlaveTransferCreateHandleDMA(SPI_SLAVE, &s_handle, NULL, NULL, &txHandle, &rxHandle);

	GPIO_PinInit(GPIOA, 7U, &Data_RDY_config);
	GPIO_PortSet(GPIOA, 1<<7U);
}

/*
 * @brief This function is used simulate a small delay.
 */
void delay(void)
{
	volatile uint32_t i = 0;
	for (i = 0; i < 50 ; ++i)
	{
		__asm("NOP"); /* delay */
	}
}

/*!
 * @brief Init for CRC-16-CCITT.
 * @details Init CRC peripheral module for CRC-16/CCITT-FALSE protocol:
 *          width=16 poly=0x1021 init=0xffff refin=false refout=false xorout=0x0000 check=0x29b1
 *          http://reveng.sourceforge.net/crc-catalogue/
 * name="CRC-16/CCITT-FALSE"
 */
static void InitCrc16_CcittFalse(CRC_Type *base, uint32_t seed)
{
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

/*
 * @brief This handler is used to CRC value of a data buffer.
 */
uint32_t GetCRC( uint32_t *data, uint32_t size, uint16_t *checksum16)
{
	/* ***************
	 * CRC-16/CCITT-FALSE *
	 *************** */
	InitCrc16_CcittFalse(base, 0xFFFFU);
	CRC_WriteData(base, (uint8_t *)data, size);
	*checksum16 = CRC_Get16bitResult(base);
}

/*
 * @brief This handler is used to prepare a data packet with header, data and CRC value.
 */
void PrepareSPIPacket()
{
	/* L1 voltage */
	sendBuff[5] = ui_buff[0] >> 24 ;
	sendBuff[4] = ui_buff[0] >> 16 ;
	sendBuff[7] = ui_buff[0] >> 8 ;
	sendBuff[6] = ui_buff[0] >> 0 ;

	/* L2 voltage */
	sendBuff[9] = ui_buff[1] >> 24 ;
	sendBuff[8] = ui_buff[1] >> 16 ;
	sendBuff[11] = ui_buff[1] >> 8 ;
	sendBuff[10] = ui_buff[1] >> 0 ;

	/* L3 voltage */
	sendBuff[13] = ui_buff[2] >> 24 ;
	sendBuff[12] = ui_buff[2] >> 16 ;
	sendBuff[15] = ui_buff[2] >> 8 ;
	sendBuff[14] = ui_buff[2] >> 0 ;

	/* L1 current */
	sendBuff[17] = ui_buff[3] >> 24 ;
	sendBuff[16] = ui_buff[3] >> 16 ;
	sendBuff[19] = ui_buff[3] >> 8 ;
	sendBuff[18] = ui_buff[3] >> 0 ;
#if (DUMP_SAMPLES == 1)
	dump_ui_buff[dump_buff_index][dump_buff_count++] = ui_buff[3];

	if(dump_buff_count == 64)
	{
		dump_buff_count = 0;
		dump_buff_index ^= 1;
	}
#endif

	/* L2 current */
	sendBuff[21] = ui_buff[4] >> 24 ;
	sendBuff[20] = ui_buff[4] >> 16 ;
	sendBuff[23] = ui_buff[4] >> 8 ;
	sendBuff[22] = ui_buff[4] >> 0 ;
#if (DUMP_SAMPLES_I2 == 1)
	dump_ui_buff[dump_buff_index][dump_buff_count++] = ui_buff[4];

	if(dump_buff_count == 64)
	{
		dump_buff_count = 0;
		dump_buff_index ^= 1;
	}
#endif

	/* L3 current */
	sendBuff[25] = ui_buff[5] >> 24 ;
	sendBuff[24] = ui_buff[5] >> 16 ;
	sendBuff[27] = ui_buff[5] >> 8 ;
	sendBuff[26] = ui_buff[5] >> 0 ;
#if (DUMP_SAMPLES_I3 == 1)
	dump_ui_buff[dump_buff_index][dump_buff_count++] = ui_buff[5];

	if(dump_buff_count == 64)
	{
		dump_buff_count = 0;
		dump_buff_index ^= 1;
	}
#endif


	/* filling 4Bytes CRC value at the end */
	GetCRC(ui_buff,sizeof(ui_buff), &g_crc_val);
	sendBuff[29] = g_crc_val >> 24 ;
	sendBuff[28] = g_crc_val >> 16 ;
	sendBuff[31] = g_crc_val >> 8 ;
	sendBuff[30] = g_crc_val >> 0 ;
}


/*
 * @brief This is the interrupt callback function for SAR ADC module.
 */
void SARADCCallback(void)
{
	if ((ADC0->SC1[0] & ADC_SC1_COCO_MASK))
	{
		ui_buff[0] = (uint16_t)ADC16_GetChannelConversionValue(ADC0, 0U);
		/* read the SD-AFE current channel sample */
		ui_buff[3] = AFE->RR[0];
	}

	if ((ADC0->SC1[1] & ADC_SC1_COCO_MASK))
	{
		ui_buff[1] = (uint16_t)ADC16_GetChannelConversionValue(ADC0, 1U);
		/* read the SD-AFE current channel sample */
		ui_buff[4] = AFE->RR[1];
	}

	if ((ADC0->SC1[2] & ADC_SC1_COCO_MASK) && (ADC0->SC1[2] & ADC_SC1_AIEN_MASK))
	{
		ui_buff[2] = (uint16_t)ADC16_GetChannelConversionValue(ADC0, 2U);
		/* read the SD-AFE current channel sample */
		ui_buff[5] = AFE->RR[2];

		read_adc_val = true;

		PrepareSPIPacket();
		xfer.txData   = sendBuff;
		xfer.rxData   = buff;
		xfer.dataSize = BUFFER_SIZE;

		SPI_SlaveTransferDMA(SPI_SLAVE, &s_handle, &xfer);

		/* toggle DRDY signal */
		GPIO_PortToggle(GPIOA, 1u << 7);
		delay();
		GPIO_PortToggle(GPIOA, 1u << 7);
	}

	SDK_ISR_EXIT_BARRIER;
}

/*
/*
 * Timer handler to sequence LED blink Rate and LCD refresh rate
 */
void LPTMR0_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);

    if(read_adc_val)
    {
    	read_adc_val = false;

    	// Initialize SPI pins if AFE is running
    	if(!spi_pin_init)
    	{
    		spi_pin_init = true;
    		/* SPI1 pins enable */
    		PORT_SetPinMux(PORTG, 2U, kPORT_MuxAlt2);
    		PORT_SetPinMux(PORTG, 3U, kPORT_MuxAlt2);
    		PORT_SetPinMux(PORTG, 4U, kPORT_MuxAlt2);
    		PORT_SetPinMux(PORTG, 5U, kPORT_MuxAlt2);
    	}
    }
}

/*
 * LPTMR is used to create refresh timing for the onboard LCD
 */
static void Start_Timer(void)
{
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
    LPTMR_GetDefaultConfig(&g_lptmrConfig);
    /* Initialize the lptmr */
    LPTMR_Init(LPTMR0, &g_lptmrConfig);

    /* Set timer period */
    /* set LPTMR to 1 Second interval */
    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(750000u, LPTMR_CLK_FREQ));
    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    //EnableIRQ(LPTMR0_LPTMR1_IRQn);
    EnableIRQWithPriority(LPTMR0_LPTMR1_IRQn, 2U);
    LPTMR_StartTimer(LPTMR0);
}

/*
 * @brief This function is used to connect AFE channel conversion complete flags to SAR-ADC triggers using XBAR.
 */
void ConnectAFEtoSARADC(void)
{
	/* Configure the XBAR signal connections. */
	XBAR_SetSignalsConnection(XBAR, kXBAR_InputAfeCh0CocFlag, kXBAR_OutputAdcTrgA);
	XBAR_SetSignalsConnection(XBAR, kXBAR_InputAfeCh1CocFlag, kXBAR_OutputAdcTrgB);
	XBAR_SetSignalsConnection(XBAR, kXBAR_InputAfeCh2CocFlag, kXBAR_OutputAdcTrgC);
}

/*
 * @brief This handler is used to initialize SARADC, 4 channels of SAR ADC have been enabled with one of them generating interrupts.
 */
void InitSARADC(void)
{
	adc16_config_t adc16ConfigStruct;
	adc16_channel_config_t adc16ChannelConfigStruct;

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

	adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	adc16ConfigStruct.clockSource = kADC16_ClockSourceAlt1;
	adc16ConfigStruct.enableAsynchronousClock = false;
	adc16ConfigStruct.clockDivider = kADC16_ClockDivider4;
	adc16ConfigStruct.resolution = kADC16_ResolutionSE16Bit;
	adc16ConfigStruct.enableHighSpeed = true ;
	ADC16_Init(DEMO_ADC16_BASEADDR, &adc16ConfigStruct);

	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASEADDR, true);
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	/* Configure channel A : L1 Voltage sense channel */
	adc16ChannelConfigStruct.channelNumber = 0U;
	ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, 0U, &adc16ChannelConfigStruct);
	/* Configure channel B : L2 Voltage sense channel */
	adc16ChannelConfigStruct.channelNumber = 1U;
	ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, 1U, &adc16ChannelConfigStruct);
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;
	/* Configure channel C : L3 Voltage sense channel */
	adc16ChannelConfigStruct.channelNumber = 2U;
	ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, 2U, &adc16ChannelConfigStruct);
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

	ADC16_EnableDMA(DEMO_ADC16_BASEADDR, false);
	NVIC_SetPriority(DEMO_ADC16_IRQN, 1U);
	NVIC_EnableIRQ(DEMO_ADC16_IRQN);
}

/*!
 * @brief Iniitializes VREF module so that the reference voltage can be used by other
 * modules(e.g., Comparator with line voltage to generate an o/p to triggger the timer
 * for frequency calculation.
 */
void InitVREF(void)
{
	uint32_t temp32;
	/* Do necessary initialization in the SIM module */
	temp32 = SIM->MISC_CTL & ~(SIM_MISC_CTL_VREFBUFPD_MASK | SIM_MISC_CTL_VREFBUFINSEL_MASK | SIM_MISC_CTL_VREFBUFOUTEN_MASK);
	temp32 |= SIM_MISC_CTL_VREFBUFPD(0) | SIM_MISC_CTL_VREFBUFINSEL(0) | SIM_MISC_CTL_VREFBUFOUTEN(1);
	SIM->MISC_CTL = temp32;

	/* VREF module must be initialized after SIM module                         */
	vref_config_t config;

	/* Get vref default configure */
	VREF_GetDefaultConfig(&config);
#if defined(FSL_FEATURE_VREF_HAS_LOW_REFERENCE) && FSL_FEATURE_VREF_HAS_LOW_REFERENCE
	/* Enable low reference volt */
	config.enableLowRef = true;
#endif /* FSL_FEATURE_VREF_HAS_LOW_REFERENCE */
	/* Initialize vref */
	VREF_Init(VREF, &config);
}

/*!
 * @brief Initializes AFE channels for voltage and current channels.
 */
void InitAFE(void)
{
	uint8_t i=0;
	afe_channel_config_t afeChnConfig;
	afe_config_t afeConfig;

	afeConfig.enableLowPower = true;
	afeConfig.resultFormat = kAFE_ResultFormatRight;
	afeConfig.clockSource = kAFE_ClockSource3;
	afeConfig.clockDivider = kAFE_ClockDivider1;
	afeConfig.startupCount = 15U; /* startupCnt = (Clk_freq/Clk_div)*20e-6 */

	AFE_Init(AFE, &afeConfig);

	for(i=0;i<0xff;i++);		/* add little start-up delay */

	afeChnConfig.enableHardwareTrigger      = false;
	afeChnConfig.enableContinuousConversion = true;
	afeChnConfig.channelMode                = kAFE_BypassDisable;
	afeChnConfig.decimatorOversampleRatio   = kAFE_DecimatorOversampleRatio1024;
	afeChnConfig.pgaGainSelect              = kAFE_PgaDisable;
	/* Initialize AFE to emulate to measure a Phase current */
	AFE_SetChannelConfig(AFE, 0U, &afeChnConfig);
	AFE_SetChannelPhaseDelayValue(AFE, 0U, (0 * SAR_SCT));
	/* Initialize AFE to emulate to measure a Phase current */
	AFE_SetChannelConfig(AFE, 1U, &afeChnConfig);
	AFE_SetChannelPhaseDelayValue(AFE, 1U, (1 * SAR_SCT));
	/* Initialize AFE to emulate to measure a Phase current */
	AFE_SetChannelConfig(AFE, 2U, &afeChnConfig);
	AFE_SetChannelPhaseDelayValue(AFE, 2U, (2 * SAR_SCT));
}


/*!
 * @brief Initializes the MCU resources used to aid the core metrology calculation.
 */
void MeteringInaterfaceInit(void)
{
	mcg_pll_config_t pllConfig;

	pllConfig.refSrc = kMCG_PllRefRtc;
	pllConfig.enableMode = 0U;
	CLOCK_EnablePll0(&pllConfig);

	XBAR_Init(XBAR);

	ConnectAFEtoSARADC();
	InitSARADC();
	InitVREF();

	/* Now trigger the AFE channels after configuration */
	InitAFE();
	AFE_DoSoftwareTriggerChannel(AFE, (AFE_CR_SOFT_TRG0_MASK | AFE_CR_SOFT_TRG1_MASK | AFE_CR_SOFT_TRG2_MASK));
}

/*!
 * @brief Main function
 */
int main(void)
{
	/* Define the init structure for the output LED pin */
	gpio_pin_config_t gpio_in_config = {
			kGPIO_DigitalInput,
			0,
	};
	/* Board pin init */
	BOARD_InitHardware();

	GPIO_PinInit(GPIOA, 6U, &gpio_in_config);

    Start_Timer();
	/* Initialize SPI, DMA, Data Ready Pin */
	SPI_DMA_init();

	/* Initialize all metering specific MCU IPs */
	MeteringInaterfaceInit();

	while (1)
	{
		__asm("NOP");
	}
}

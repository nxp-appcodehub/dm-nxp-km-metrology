//*****************************************************************************
// MKM35Z7 startup code for use with MCUXpresso IDE
//
// Version : 130120
//*****************************************************************************
//
// Copyright 2016-2020 NXP
// All rights reserved.
//
// SPDX-License-Identifier: BSD-3-Clause
//*****************************************************************************

#if defined (DEBUG)
#pragma GCC push_options
#pragma GCC optimize ("Og")
#endif // (DEBUG)

#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define WEAK_AV __attribute__ ((weak, section(".after_vectors")))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************
// Flash Configuration block : 16-byte flash configuration field that stores
// default protection settings (loaded on reset) and security information that
// allows the MCU to restrict access to the Flash Memory module.
// Placed at address 0x400 by the linker script.
//*****************************************************************************
__attribute__ ((used,section(".FlashConfig"))) const struct {
    unsigned int word1;
    unsigned int word2;
    unsigned int word3;
    unsigned int word4;
} Flash_Config = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE};
//*****************************************************************************
// Declaration of external SystemInit function
//*****************************************************************************
#if defined (__USE_CMSIS)
extern void SystemInit(void);
#endif // (__USE_CMSIS)

//*****************************************************************************
// Forward declaration of the core exception handlers.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions.
// If your application is a C++ one, then any interrupt handlers defined
// in C++ files within in your main application will need to have C linkage
// rather than C++ linkage. To do this, make sure that you are using extern "C"
// { .... } around the interrupt handler within your main application code.
//*****************************************************************************
     void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

//*****************************************************************************
// Forward declaration of the application IRQ handlers. When the application
// defines a handler (with the same name), this will automatically take
// precedence over weak definitions below
//*****************************************************************************
WEAK void DMA0_IRQHandler(void);
WEAK void DMA1_IRQHandler(void);
WEAK void DMA2_IRQHandler(void);
WEAK void DMA3_IRQHandler(void);
WEAK void SPI0_SPI1_SPI2_IRQHandler(void);
WEAK void PDB0_IRQHandler(void);
WEAK void PMC_IRQHandler(void);
WEAK void TMR0_IRQHandler(void);
WEAK void TMR1_IRQHandler(void);
WEAK void TMR2_IRQHandler(void);
WEAK void TMR3_IRQHandler(void);
WEAK void PIT0_PIT1_IRQHandler(void);
WEAK void LLWU_IRQHandler(void);
WEAK void FTFA_IRQHandler(void);
WEAK void CMP0_CMP1_CMP2_IRQHandler(void);
WEAK void LCD_IRQHandler(void);
WEAK void ADC0_IRQHandler(void);
WEAK void PTx_IRQHandler(void);
WEAK void RNGA_IRQHandler(void);
WEAK void UART0_UART1_UART2_UART3_IRQHandler(void);
WEAK void MMAU_IRQHandler(void);
WEAK void AFE_CH0_IRQHandler(void);
WEAK void AFE_CH1_IRQHandler(void);
WEAK void AFE_CH2_IRQHandler(void);
WEAK void AFE_CH3_IRQHandler(void);
WEAK void RTC_IRQHandler(void);
WEAK void I2C0_I2C1_IRQHandler(void);
WEAK void LPUART0_IRQHandler(void);
WEAK void MCG_IRQHandler(void);
WEAK void WDOG_EWM_IRQHandler(void);
WEAK void LPTMR0_LPTMR1_IRQHandler(void);
WEAK void XBAR_IRQHandler(void);

//*****************************************************************************
// Forward declaration of the driver IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the driver
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//*****************************************************************************
void DMA0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SPI0_SPI1_SPI2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PDB0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PMC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void TMR0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void TMR1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void TMR2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void TMR3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIT0_PIT1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LLWU_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FTFA_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMP0_CMP1_CMP2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LCD_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PTx_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RNGA_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_UART1_UART2_UART3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void MMAU_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void AFE_CH0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void AFE_CH1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void AFE_CH2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void AFE_CH3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I2C0_I2C1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPUART0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void MCG_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void WDOG_EWM_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPTMR0_LPTMR1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void XBAR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//*****************************************************************************
#if defined (__REDLIB__)
extern void __main(void);
#endif
extern int main(void);

//*****************************************************************************
// External declaration for the pointer to the stack top from the Linker Script
//*****************************************************************************
extern void _vStackTop(void);
//*****************************************************************************
#if defined (__cplusplus)
} // extern "C"
#endif
//*****************************************************************************
// The vector table.
// This relies on the linker script to place at correct location in memory.
//*****************************************************************************



extern void (* const g_pfnVectors[])(void);
extern void * __Vectors __attribute__ ((alias ("g_pfnVectors")));

__attribute__ ((used, section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core Level - CM0P
    &_vStackTop,                       // The initial stack pointer
    ResetISR,                          // The reset handler
    NMI_Handler,                       // The NMI handler
    HardFault_Handler,                 // The hard fault handler
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    SVC_Handler,                       // SVCall handler
    0,                                 // Reserved
    0,                                 // Reserved
    PendSV_Handler,                    // The PendSV handler
    SysTick_Handler,                   // The SysTick handler

    // Chip Level - MKM35Z7
    DMA0_IRQHandler,                     // 16: DMA channel 0 transfer complete
    DMA1_IRQHandler,                     // 17: DMA channel 1 transfer complete
    DMA2_IRQHandler,                     // 18: DMA channel 2 transfer complete
    DMA3_IRQHandler,                     // 19: DMA channel 3 transfer complete
    SPI0_SPI1_SPI2_IRQHandler,           // 20: SPI0/SPI1/SPI2 ORed interrupt
    PDB0_IRQHandler,                     // 21: PDB0 ORed interrupt
    PMC_IRQHandler,                      // 22: Low-voltage detect, low-voltage warning
    TMR0_IRQHandler,                     // 23: Quad Timer Channel 0
    TMR1_IRQHandler,                     // 24: Quad Timer Channel 1
    TMR2_IRQHandler,                     // 25: Quad Timer Channel 2
    TMR3_IRQHandler,                     // 26: Quad Timer Channel 3
    PIT0_PIT1_IRQHandler,                // 27: PIT0/PIT1 ORed interrupt
    LLWU_IRQHandler,                     // 28: Low Leakage Wakeup
    FTFA_IRQHandler,                     // 29: Command complete and read collision
    CMP0_CMP1_CMP2_IRQHandler,           // 30: CMP0/CMP1/CMP2 ORed interrupt
    LCD_IRQHandler,                      // 31: LCD interrupt
    ADC0_IRQHandler,                     // 32: ADC0 interrupt
    PTx_IRQHandler,                      // 33: Single interrupt vector for GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG,GPIOH,GPIOI,GPIOJ,GPIOK,GPIOL,GPIOM
    RNGA_IRQHandler,                     // 34: RNGA interrupt
    UART0_UART1_UART2_UART3_IRQHandler,  // 35: UART0/UART1/UART2/UART3 ORed interrupt
    MMAU_IRQHandler,                     // 36: Memory Mapped Arithmetic Unit interrupt
    AFE_CH0_IRQHandler,                  // 37: AFE Channel 0
    AFE_CH1_IRQHandler,                  // 38: AFE Channel 1
    AFE_CH2_IRQHandler,                  // 39: AFE Channel 2
    AFE_CH3_IRQHandler,                  // 40: AFE Channel 3
    RTC_IRQHandler,                      // 41: IRTC interrupt
    I2C0_I2C1_IRQHandler,                // 42: I2C0/I2C1 ORed interrupt
    LPUART0_IRQHandler,                  // 43: LPUART0 status and error interrupt
    MCG_IRQHandler,                      // 44: MCG interrupt
    WDOG_EWM_IRQHandler,                 // 45: WDOG/EWM ORed interrupt
    LPTMR0_LPTMR1_IRQHandler,            // 46: LPTMR0/LPTMR1 interrupt
    XBAR_IRQHandler,                     // 47: XBAR interrupt


}; /* End of g_pfnVectors */

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors.init_data")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors.init_bss")))
void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((section(".after_vectors.reset")))
void ResetISR(void) {

    // Disable interrupts
    __asm volatile ("cpsid i");


#if defined (__USE_CMSIS)
// If __USE_CMSIS defined, then call CMSIS SystemInit code
    SystemInit();

#else
    // Disable Watchdog
    //  Write 0xC520 to watchdog unlock register
    *((volatile unsigned short *)0x4005300E) = 0xC520;
    //  Followed by 0xD928 to complete the unlock
    *((volatile unsigned short *)0x4005300E) = 0xD928;
    // Now disable watchdog via STCTRLH register
    *((volatile unsigned short *)0x40053000) = 0x01D2u;
#endif // (__USE_CMSIS)

    //
    // Copy the data sections from flash to SRAM.
    //
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }

    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }


#if !defined (__USE_CMSIS)
// Assume that if __USE_CMSIS defined, then CMSIS SystemInit code
// will setup the VTOR register

    // Check to see if we are running the code from a non-zero
    // address (eg RAM, external flash), in which case we need
    // to modify the VTOR register to tell the CPU that the
    // vector table is located at a non-0x0 address.
    unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
    if ((unsigned int *)g_pfnVectors!=(unsigned int *) 0x00000000) {
        *pSCB_VTOR = (unsigned int)g_pfnVectors;
    }
#endif // (__USE_CMSIS)
#if defined (__cplusplus)
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
#endif

    // Reenable interrupts
    __asm volatile ("cpsie i");

#if defined (__REDLIB__)
    // Call the Redlib library, which in turn calls main()
    __main();
#else
    main();
#endif

    //
    // main() shouldn't return, but if it does, we'll just enter an infinite loop
    //
    while (1) {
        ;
    }
}

//*****************************************************************************
// Default core exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
WEAK_AV void NMI_Handler(void)
{ while(1) {}
}

WEAK_AV void HardFault_Handler(void)
{ while(1) {}
}

WEAK_AV void SVC_Handler(void)
{ while(1) {}
}

WEAK_AV void PendSV_Handler(void)
{ while(1) {}
}

WEAK_AV void SysTick_Handler(void)
{ while(1) {}
}

//*****************************************************************************
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//*****************************************************************************
WEAK_AV void IntDefaultHandler(void)
{ while(1) {}
}

//*****************************************************************************
// Default application exception handlers. Override the ones here by defining
// your own handler routines in your application code. These routines call
// driver exception handlers or IntDefaultHandler() if no driver exception
// handler is included.
//*****************************************************************************
WEAK_AV void DMA0_IRQHandler(void)
{   DMA0_DriverIRQHandler();
}

WEAK_AV void DMA1_IRQHandler(void)
{   DMA1_DriverIRQHandler();
}

WEAK_AV void DMA2_IRQHandler(void)
{   DMA2_DriverIRQHandler();
}

WEAK_AV void DMA3_IRQHandler(void)
{   DMA3_DriverIRQHandler();
}

WEAK_AV void SPI0_SPI1_SPI2_IRQHandler(void)
{   SPI0_SPI1_SPI2_DriverIRQHandler();
}

WEAK_AV void PDB0_IRQHandler(void)
{   PDB0_DriverIRQHandler();
}

WEAK_AV void PMC_IRQHandler(void)
{   PMC_DriverIRQHandler();
}

WEAK_AV void TMR0_IRQHandler(void)
{   TMR0_DriverIRQHandler();
}

WEAK_AV void TMR1_IRQHandler(void)
{   TMR1_DriverIRQHandler();
}

WEAK_AV void TMR2_IRQHandler(void)
{   TMR2_DriverIRQHandler();
}

WEAK_AV void TMR3_IRQHandler(void)
{   TMR3_DriverIRQHandler();
}

WEAK_AV void PIT0_PIT1_IRQHandler(void)
{   PIT0_PIT1_DriverIRQHandler();
}

WEAK_AV void LLWU_IRQHandler(void)
{   LLWU_DriverIRQHandler();
}

WEAK_AV void FTFA_IRQHandler(void)
{   FTFA_DriverIRQHandler();
}

WEAK_AV void CMP0_CMP1_CMP2_IRQHandler(void)
{   CMP0_CMP1_CMP2_DriverIRQHandler();
}

WEAK_AV void LCD_IRQHandler(void)
{   LCD_DriverIRQHandler();
}

WEAK_AV void ADC0_IRQHandler(void)
{   ADC0_DriverIRQHandler();
}

WEAK_AV void PTx_IRQHandler(void)
{   PTx_DriverIRQHandler();
}

WEAK_AV void RNGA_IRQHandler(void)
{   RNGA_DriverIRQHandler();
}

WEAK_AV void UART0_UART1_UART2_UART3_IRQHandler(void)
{   UART0_UART1_UART2_UART3_DriverIRQHandler();
}

WEAK_AV void MMAU_IRQHandler(void)
{   MMAU_DriverIRQHandler();
}

WEAK_AV void AFE_CH0_IRQHandler(void)
{   AFE_CH0_DriverIRQHandler();
}

WEAK_AV void AFE_CH1_IRQHandler(void)
{   AFE_CH1_DriverIRQHandler();
}

WEAK_AV void AFE_CH2_IRQHandler(void)
{   AFE_CH2_DriverIRQHandler();
}

WEAK_AV void AFE_CH3_IRQHandler(void)
{   AFE_CH3_DriverIRQHandler();
}

WEAK_AV void RTC_IRQHandler(void)
{   RTC_DriverIRQHandler();
}

WEAK_AV void I2C0_I2C1_IRQHandler(void)
{   I2C0_I2C1_DriverIRQHandler();
}

WEAK_AV void LPUART0_IRQHandler(void)
{   LPUART0_DriverIRQHandler();
}

WEAK_AV void MCG_IRQHandler(void)
{   MCG_DriverIRQHandler();
}

WEAK_AV void WDOG_EWM_IRQHandler(void)
{   WDOG_EWM_DriverIRQHandler();
}

WEAK_AV void LPTMR0_LPTMR1_IRQHandler(void)
{   LPTMR0_LPTMR1_DriverIRQHandler();
}

WEAK_AV void XBAR_IRQHandler(void)
{   XBAR_DriverIRQHandler();
}

//*****************************************************************************

#if defined (DEBUG)
#pragma GCC pop_options
#endif // (DEBUG)

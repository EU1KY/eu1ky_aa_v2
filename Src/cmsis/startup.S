/************************************************************************************
 * File:    startup.S
 * Purpose: Startup file for Cortex-M3 devices.
 *          Should use with GCC for ARM Embedded Processors
 * Version: V1.4
 * Date:    09 July 2012
 * Notice:  Changed for use with emIDE project wizard
 * Date:    05 July 2013
 *
 * Copyright (c) 2011, 2012, ARM Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the ARM Limited nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARM LIMITED BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
    .syntax unified
    .arch armv7-m

/************************************************************************************
 *
 *  The minimal vector table for Cortex-M3.
 *  Device specific external interrupts can be added below.
 ************************************************************************************/
    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __stack_end__         /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    .long    WWDG_IRQHandler              /*!<  0: Window Watchdog                      */
    .long    PVD_IRQHandler               /*!<  1: PVD through EXTI Line detect         */
    .long    TAMPER_IRQHandler            /*!<  2: Tamper                               */
    .long    RTC_IRQHandler               /*!<  3: RTC                                  */
    .long    FLASH_IRQHandler             /*!<  4: Flash                                */
    .long    RCC_IRQHandler               /*!<  5: RCC                                  */
    .long    EXTI0_IRQHandler             /*!<  6: EXTI Line 0                          */
    .long    EXTI1_IRQHandler             /*!<  7: EXTI Line 1                          */
    .long    EXTI2_IRQHandler             /*!<  8: EXTI Line 2                          */
    .long    EXTI3_IRQHandler             /*!<  9: EXTI Line 3                          */
    .long    EXTI4_IRQHandler             /*!< 10: EXTI Line 4                          */
    .long    DMA1_Channel1_IRQHandler     /*!< 11: DMA1 Channel 1                       */
    .long    DMA1_Channel2_IRQHandler     /*!< 12: DMA1 Channel 2                       */
    .long    DMA1_Channel3_IRQHandler     /*!< 13: DMA1 Channel 3                       */
    .long    DMA1_Channel4_IRQHandler     /*!< 14: DMA1 Channel 4                       */
    .long    DMA1_Channel5_IRQHandler     /*!< 15: DMA1 Channel 5                       */
    .long    DMA1_Channel6_IRQHandler     /*!< 16: DMA1 Channel 6                       */
    .long    DMA1_Channel7_IRQHandler     /*!< 17: DMA1 Channel 7                       */
    .long    ADC1_2_IRQHandler            /*!< 18: ADC1 & ADC2                          */
    .long    USB_HP_CAN1_TX_IRQHandler    /*!< 19: USB High Priority or CAN1 TX         */
    .long    USB_LP_CAN1_RX0_IRQHandler   /*!< 20: USB Low  Priority or CAN1 RX0        */
    .long    CAN1_RX1_IRQHandler          /*!< 21: CAN1 RX1                             */
    .long    CAN1_SCE_IRQHandler          /*!< 22: CAN1 SCE                             */
    .long    EXTI9_5_IRQHandler           /*!< 23: EXTI Line 9..5                       */
    .long    TIM1_BRK_IRQHandler          /*!< 24: TIM1 Break                           */
    .long    TIM1_UP_IRQHandler           /*!< 25: TIM1 Update                          */
    .long    TIM1_TRG_COM_IRQHandler      /*!< 26: TIM1 Trigger and Commutation         */
    .long    TIM1_CC_IRQHandler           /*!< 27: TIM1 Capture Compare                 */
    .long    TIM2_IRQHandler              /*!< 28: TIM2                                 */
    .long    TIM3_IRQHandler              /*!< 29: TIM3                                 */
    .long    TIM4_IRQHandler              /*!< 30: TIM4                                 */
    .long    I2C1_EV_IRQHandler           /*!< 31: I2C1 Event                           */
    .long    I2C1_ER_IRQHandler           /*!< 32: I2C1 Error                           */
    .long    I2C2_EV_IRQHandler           /*!< 33: I2C2 Event                           */
    .long    I2C2_ER_IRQHandler           /*!< 34: I2C2 Error                           */
    .long    SPI1_IRQHandler              /*!< 35: SPI1                                 */
    .long    SPI2_IRQHandler              /*!< 36: SPI2                                 */
    .long    USART1_IRQHandler            /*!< 37: USART1                               */
    .long    USART2_IRQHandler            /*!< 38: USART2                               */
    .long    USART3_IRQHandler            /*!< 39: USART3                               */
    .long    EXTI15_10_IRQHandler         /*!< 40: EXTI Line 15..10                     */
    .long    RTCAlarm_IRQHandler          /*!< 41: RTC Alarm through EXTI Line          */
    .long    USBWakeUp_IRQHandler         /*!< 42: USB Wakeup from suspend              */
    .long    TIM8_BRK_IRQHandler          /*!< 43: TIM8 Break                           */
    .long    TIM8_UP_IRQHandler           /*!< 44: TIM8 Update                          */
    .long    TIM8_TRG_COM_IRQHandler      /*!< 45: TIM8 Trigger and Commutation         */
    .long    TIM8_CC_IRQHandler           /*!< 46: TIM8 Capture Compare                 */
    .long    ADC3_IRQHandler              /*!< 47: ADC3                                 */
    .long    FSMC_IRQHandler              /*!< 48: FSMC                                 */
    .long    SDIO_IRQHandler              /*!< 49: SDIO                                 */
    .long    TIM5_IRQHandler              /*!< 50: TIM5                                 */
    .long    SPI3_IRQHandler              /*!< 51: SPI3                                 */
    .long    UART4_IRQHandler             /*!< 52: UART4                                */
    .long    UART5_IRQHandler             /*!< 52: UART5                                */
    .long    TIM6_IRQHandler              /*!< 53: TIM6                                 */
    .long    TIM7_IRQHandler              /*!< 54: TIM7                                 */
    .long    DMA2_Channel1_IRQHandler     /*!< 55: DMA2 Channel1                        */
    .long    DMA2_Channel2_IRQHandler     /*!< 56: DMA2 Channel2                        */
    .long    DMA2_Channel3_IRQHandler     /*!< 57: DMA2 Channel3                        */
    .long    DMA2_Channel4_5_IRQHandler   /*!< 58: DMA2 Channel4 & Channel5             */

    /* External interrupts */
    .long    Default_Handler

    .size    __isr_vector, . - __isr_vector

/************************************************************************************
 *
 *  Reset_Handler()
 *  This function gets called at start of execution after a reset event.
 *  Copies data from ROM to RAM, clears BSS if defined,
 *  calls SystemInit() if defined, finally calls main()
 ************************************************************************************/
    .text
    .thumb
    .global __stack_end__
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:

/*  Setup stack pointer. Helpful for targets running in RAM without script file */
    ldr   r1, =__stack_end__
    msr   msp, r1

/*  Loop to copy data from read only memory to RAM. The ranges
 *   of copy from/to are specified by following symbols evaluated in
 *   linker script.
 *   __etext: End of code section, i.e., begin of data sections to copy from.
 *   __data_start__/__data_end__: RAM address range that data should be
 *   copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

#if 1
/*  Here are two copies of loop implemenations. First one favors code size
 *  and the second one favors performance. Default uses the first one.
 *  Change to "#if 0" to use the second one */
.LC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0
#else
    subs    r3, r2
    ble    .LC1
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:
#endif

/*  Loop to zero out BSS section, which uses following symbols
 *  in linker script:
 *   __bss_start__: start of BSS section. Must align to 4
 *   __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    movs    r0, 0
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2

    bl __libc_init_array
#ifndef __NO_SYSTEM_INIT
    bl    SystemInit
#endif

    bl    main
    bl __libc_fini_array
    .pool
    .size Reset_Handler, . - Reset_Handler

/************************************************************************************
 *  Default_Handler()
 ************************************************************************************/
    .text
    .thumb
    .thumb_func
    .align 2
    .global    Default_Handler
    .type    Default_Handler, %function
Default_Handler:
    // Load the address of the interrupt control register into r3.
    ldr r3,=0xe000ed04 //The address of the NVIC interrupt control register
    // Load the value of the interrupt control register into r2 from the address held in r3.
    ldr r2, [r3, #0]
    // The interrupt number is in the least significant byte - clear all other bits.
    uxtb r2, r2
    // Now sit in an infinite loop - the number of the executing interrupt is held in r2.
    b  .
    .pool
    .size  Default_Handler, .-Default_Handler

/************************************************************************************
 *  HardFault_Handler()
 ************************************************************************************/
    .text
    .thumb
    .thumb_func
    .align 2
    .global    HardFault_Handler
    .type    HardFault_Handler, %function
HardFault_Handler:
    tst lr, #4
    ite eq
    mrseq r0, msp
    mrsne r0, psp
    ldr r1, [r0, #24] //R1 points to array of 8 registers? r0 r1 r2 r3 r12 lr pc psr
    //ldr r2,=GetRegistersFromStack
    //bx r2
    b .
    .pool
    .size  HardFault_Handler, .-HardFault_Handler

/************************************************************************************
 *
 *  Weak definition for exceptions.
 *  Any function with the same name will override the weak definition.
 ************************************************************************************/
/*  Macro to define default handlers. Default handler
 *  will be weak symbol and just dead loops. They can be
 *  overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_default_handler    NMI_Handler
    //def_default_handler    HardFault_Handler
    def_default_handler    MemManage_Handler
    def_default_handler    BusFault_Handler
    def_default_handler    UsageFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    DebugMon_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    //def_default_handler    Default_Handler

    .weak    DEF_IRQHandler
    .set    DEF_IRQHandler, Default_Handler

    def_default_handler    WWDG_IRQHandler
    def_default_handler    PVD_IRQHandler
    def_default_handler    TAMPER_IRQHandler
    def_default_handler    RTC_IRQHandler
    def_default_handler    FLASH_IRQHandler
    def_default_handler    RCC_IRQHandler
    def_default_handler    EXTI0_IRQHandler
    def_default_handler    EXTI1_IRQHandler
    def_default_handler    EXTI2_IRQHandler
    def_default_handler    EXTI3_IRQHandler
    def_default_handler    EXTI4_IRQHandler
    def_default_handler    DMA1_Channel1_IRQHandler
    def_default_handler    DMA1_Channel2_IRQHandler
    def_default_handler    DMA1_Channel3_IRQHandler
    def_default_handler    DMA1_Channel4_IRQHandler
    def_default_handler    DMA1_Channel5_IRQHandler
    def_default_handler    DMA1_Channel6_IRQHandler
    def_default_handler    DMA1_Channel7_IRQHandler
    def_default_handler    ADC1_2_IRQHandler
    def_default_handler    USB_HP_CAN1_TX_IRQHandler
    def_default_handler    USB_LP_CAN1_RX0_IRQHandler
    def_default_handler    CAN1_RX1_IRQHandler
    def_default_handler    CAN1_SCE_IRQHandler
    def_default_handler    EXTI9_5_IRQHandler
    def_default_handler    TIM1_BRK_IRQHandler
    def_default_handler    TIM1_UP_IRQHandler
    def_default_handler    TIM1_TRG_COM_IRQHandler
    def_default_handler    TIM1_CC_IRQHandler
    def_default_handler    TIM2_IRQHandler
    def_default_handler    TIM3_IRQHandler
    def_default_handler    TIM4_IRQHandler
    def_default_handler    I2C1_EV_IRQHandler
    def_default_handler    I2C1_ER_IRQHandler
    def_default_handler    I2C2_EV_IRQHandler
    def_default_handler    I2C2_ER_IRQHandler
    def_default_handler    SPI1_IRQHandler
    def_default_handler    SPI2_IRQHandler
    def_default_handler    USART1_IRQHandler
    def_default_handler    USART2_IRQHandler
    def_default_handler    USART3_IRQHandler
    def_default_handler    EXTI15_10_IRQHandler
    def_default_handler    RTCAlarm_IRQHandler
    def_default_handler    USBWakeUp_IRQHandler
    def_default_handler    TIM8_BRK_IRQHandler
    def_default_handler    TIM8_UP_IRQHandler
    def_default_handler    TIM8_TRG_COM_IRQHandler
    def_default_handler    TIM8_CC_IRQHandler
    def_default_handler    ADC3_IRQHandler
    def_default_handler    FSMC_IRQHandler
    def_default_handler    SDIO_IRQHandler
    def_default_handler    TIM5_IRQHandler
    def_default_handler    SPI3_IRQHandler
    def_default_handler    UART4_IRQHandler
    def_default_handler    UART5_IRQHandler
    def_default_handler    TIM6_IRQHandler
    def_default_handler    TIM7_IRQHandler
    def_default_handler    DMA2_Channel1_IRQHandler
    def_default_handler    DMA2_Channel2_IRQHandler
    def_default_handler    DMA2_Channel3_IRQHandler
    def_default_handler    DMA2_Channel4_5_IRQHandler

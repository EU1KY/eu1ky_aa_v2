/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

 /* ===============================================================
       USER CONFIGURABLE BOARD SPECIFIC CONFIGURATION PARAMETERS
    =============================================================== */

#ifndef _CONFIG_H_INCLUDED_
#define _CONFIG_H_INCLUDED_

//Firmvare revision string
#define VER "2.13"

// Originally, Bluetooth UART (HC-06 module) used in remote control window is connected to USART3
// PB10: USART3_TX
// PB11: USART3_RX
// You can switch BTUSART to different port, e.g. USART1, by changing the definition below
// to #define USART_TO_USE 1
#define USART_TO_USE 3

#if USART_TO_USE == 3
    #define BTUSART_USED USART3
    #define BTUSART_BAUDRATE 9600
    #define BTUSART_IRQHandler USART3_IRQHandler
    #define BTUSART_RCC RCC_APB1Periph_USART3
    #define BTUSART_IRQn USART3_IRQn
    #define BTUSART_GPIO GPIOB
    #define BTUSART_GPIO_APB2 RCC_APB2Periph_GPIOB
    #define BTUSART_PIN_Tx GPIO_Pin_10
    #define BTUSART_PIN_Rx GPIO_Pin_11
#elif USART_TO_USE == 1
    #define BTUSART_USED USART1
    #define BTUSART_BAUDRATE 38400
    #define BTUSART_IRQHandler USART1_IRQHandler
    #define BTUSART_RCC RCC_APB2Periph_USART1
    #define BTUSART_IRQn USART1_IRQn
    #define BTUSART_GPIO GPIOA
    #define BTUSART_GPIO_APB2 RCC_APB2Periph_GPIOA
    #define BTUSART_PIN_Tx GPIO_Pin_9
    #define BTUSART_PIN_Rx GPIO_Pin_10
#else
    #error USART_TO_USE must be either 1 or 3
#endif


//Frequency range of the analyzer
#define BAND_FMAX 200000000ul //BAND_FMAX must be multiple of 100000
#define BAND_FMIN 100000ul   //BAND_FMIN must be 100000

#if (BAND_FMAX % 100000) != 0 || BAND_FMAX < BAND_FMIN || BAND_FMIN != 100000
#error "Incorrect band limit settings"
#endif

//If quadrature mixer is used (RF2713), define F_LO_DIVIDED_BY_TWO
//If two mixers are used without quadrature (e.g. two of AD8342), comment it out
//#define F_LO_DIVIDED_BY_TWO

//Custom XTAL frequency (in Hz) of Si5351a can be set here. Uncomment and edit if
//you are using not a 27 MHz crystal with it. Enter your crystal frequency in Hz.
//#define SI5351_XTAL_FREQ                    27000000

//Uncomment if your Si5351a has alternative I2C address (there was a batch of
//defected chips sold through Mouser, they have this address instead of documented)
//#define SI5351_BUS_BASE_ADDR 0xCE

#endif // _CONFIG_H_INCLUDED_

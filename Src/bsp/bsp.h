/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

//Board support package (BSP) used to define different LCD/Touchscreen connections

#ifndef BSP_H_INCLUDED
#define BSP_H_INCLUDED

#include <stdint.h>
#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

//BSP GPIO port descriptor
typedef struct
{
    GPIO_TypeDef    *port;
    uint16_t         pinmask;
    GPIOMode_TypeDef mode;
} BSP_GPIO;

typedef enum
{
    BSP_LCD_WR,      // LCD WR port
    BSP_LCD_RD,      // LCD RD port
    BSP_LCD_DATA1,   // LCD FSMC 16-bit data bus part 1
    BSP_LCD_DATA2,   // LCD FSMC 16-bit data bus part 2
    BSP_LCD_RS,      // LCD FSMC RS port
    BSP_LCD_CS,      // LCD FSMC CS port
    BSP_LCD_RST,     // LCD Reset port
    BSP_LCD_BL,      // LCD Backlight on/off port
    BSP_LCD_BLPOL,   // LCD backlight polarity invert pin
    BSP_LCD_TIRQ,    // LCD touchscreen interrupt pin
    BSP_LCD_TSS,     // LCD touchscreen SPI SS (CS) port
    BSP_LCD_TSCK,    // LCD touchscreen SPI SCK (TCK) port
    BSP_LCD_TMOSI,   // LCD touchscreen SPI MOSI port
    BSP_LCD_TMISO,   // LCD touchscreen SPI MISO pin
} BSP_GPIO_E;

//BSP initialization
void BSP_Init(void);

//Output 0 to BSP GPIO port
void BSP_Out_0(BSP_GPIO_E en);

//Output 1 to BSP GPIO port
void BSP_Out_1(BSP_GPIO_E en);

//Read bit from BSP GPIO port
uint32_t BSP_In(BSP_GPIO_E en);

//Delay us microseconds (blocking)
void BSP_DelayUs(uint32_t us);

//Delay ms milliseconds (blocking)
void BSP_DelayMs(uint32_t ms);

//-----------------------------------------------------------------------------------------
#if defined BSP_HY_REDBULL_V2 //HY RedBull V2 board (STM32F103ZET6 with 3.2" LCD on SSD1289 controller)
    //Board specific LCD connections
    #define BSP_LCD_WR_V    {GPIOD, GPIO_Pin_5, GPIO_Mode_AF_PP}
    #define BSP_LCD_RD_V    {GPIOD, GPIO_Pin_4, GPIO_Mode_AF_PP}
    #define BSP_LCD_DATA1_V {GPIOD, 0xC703, GPIO_Mode_AF_PP} //FSMC data bus part 1
    #define BSP_LCD_DATA2_V {GPIOE, 0xFF80, GPIO_Mode_AF_PP} //FSMC data bus part 2
    #define BSP_LCD_RS_V    {GPIOF, GPIO_Pin_0, GPIO_Mode_AF_PP}
    #define BSP_LCD_CS_V    {GPIOG, GPIO_Pin_12, GPIO_Mode_AF_PP}
    #define BSP_LCD_RST_V   {GPIOG, GPIO_Pin_15, GPIO_Mode_Out_PP} //LCD Reset is not connected actully on this board
    #define BSP_LCD_BL_V    {GPIOA, GPIO_Pin_1, GPIO_Mode_Out_PP}
    #define BSP_LCD_BLPOL_V {GPIOE, GPIO_Pin_6, GPIO_Mode_IPU}
    //Board specific Touchscreen controller connections
    #define BSP_LCD_TIRQ_V  {GPIOG, GPIO_Pin_7, GPIO_Mode_IPU}
    #define BSP_LCD_TSS_V   {GPIOB, GPIO_Pin_12, GPIO_Mode_Out_PP}
    #define BSP_LCD_TSCK_V  {GPIOB, GPIO_Pin_13, GPIO_Mode_Out_PP}
    #define BSP_LCD_TMOSI_V {GPIOB, GPIO_Pin_15, GPIO_Mode_Out_PP}
    #define BSP_LCD_TMISO_V {GPIOB, GPIO_Pin_14, GPIO_Mode_IPU}
    //Board specific LCD controller definitions
    #define BSP_LCD_NE         4  //FSMC chip select number (FSMC_NE1..FSMC_NE4) to which the LCD CS signal is connected
    #define BSP_LCD_RS_A       0  //FSMC address bus bit (FSMC_Axx) to which LCD RS signal is connected

//-----------------------------------------------------------------------------------------
#elif defined BSP_HY_MINI_VCT6 //HY Mini board (STM32F103VCT6 with 3.2" HY32D LCD on SSD1289 controller)
    //Board specific LCD connections
    #define BSP_LCD_WR_V    {GPIOD, GPIO_Pin_5, GPIO_Mode_AF_PP}
    #define BSP_LCD_RD_V    {GPIOD, GPIO_Pin_4, GPIO_Mode_AF_PP}
    #define BSP_LCD_DATA1_V {GPIOD, 0xC703, GPIO_Mode_AF_PP} //FSMC data bus part 1
    #define BSP_LCD_DATA2_V {GPIOE, 0xFF80, GPIO_Mode_AF_PP} //FSMC data bus part 2
    #define BSP_LCD_RS_V    {GPIOD, GPIO_Pin_11, GPIO_Mode_AF_PP}
    #define BSP_LCD_CS_V    {GPIOD, GPIO_Pin_7, GPIO_Mode_AF_PP}
    #define BSP_LCD_RST_V   {GPIOE, GPIO_Pin_1, GPIO_Mode_Out_PP} //Not connected on this board
    #define BSP_LCD_BL_V    {GPIOB, GPIO_Pin_5, GPIO_Mode_Out_PP} //LCD Backlight output
    #define BSP_LCD_BLPOL_V {GPIOE, GPIO_Pin_6, GPIO_Mode_IPU}    //Backlight polarity jumper
    //Board specific Touchscreen controller connections
    #define BSP_LCD_TIRQ_V  {GPIOB, GPIO_Pin_6, GPIO_Mode_IPU}
    #define BSP_LCD_TSS_V   {GPIOA, GPIO_Pin_4, GPIO_Mode_Out_PP}
    #define BSP_LCD_TSCK_V  {GPIOA, GPIO_Pin_5, GPIO_Mode_Out_PP}
    #define BSP_LCD_TMOSI_V {GPIOA, GPIO_Pin_7, GPIO_Mode_Out_PP}
    #define BSP_LCD_TMISO_V {GPIOA, GPIO_Pin_6, GPIO_Mode_IPU}
    //Board specific LCD controller definitions
    #define BSP_LCD_NE         1  //FSMC chip select number (FSMC_NE1..FSMC_NE4) to which the LCD CS signal is connected
    #define BSP_LCD_RS_A       16 //FSMC address bus bit (FSMC_Axx) to which LCD RS signal is connected
    //#define LCD_ENTRY_MODE   0x6058  //HY32D display specific custom entry mode register setting for SSD1289 controller

//-----------------------------------------------------------------------------------------
#else //Default board (MINI with STM32F103VET6 and 2.4" or 2.8" LCD)
    //Board specific LCD connections
    #define BSP_LCD_WR_V    {GPIOD, GPIO_Pin_5, GPIO_Mode_AF_PP}
    #define BSP_LCD_RD_V    {GPIOD, GPIO_Pin_4, GPIO_Mode_AF_PP}
    #define BSP_LCD_DATA1_V {GPIOD, 0xC703, GPIO_Mode_AF_PP} //FSMC data bus part 1
    #define BSP_LCD_DATA2_V {GPIOE, 0xFF80, GPIO_Mode_AF_PP} //FSMC data bus part 2
    #define BSP_LCD_RS_V    {GPIOD, GPIO_Pin_11, GPIO_Mode_AF_PP}
    #define BSP_LCD_CS_V    {GPIOD, GPIO_Pin_7, GPIO_Mode_AF_PP}
    #define BSP_LCD_RST_V   {GPIOE, GPIO_Pin_1, GPIO_Mode_Out_PP}
    #define BSP_LCD_BL_V    {GPIOD, GPIO_Pin_13, GPIO_Mode_Out_PP}
    #define BSP_LCD_BLPOL_V {GPIOE, GPIO_Pin_6, GPIO_Mode_IPU}
    //Board specific Touchscreen controller connections
    #define BSP_LCD_TIRQ_V  {GPIOB, GPIO_Pin_6, GPIO_Mode_IPU}
    #define BSP_LCD_TSS_V   {GPIOB, GPIO_Pin_7, GPIO_Mode_Out_PP}
    #define BSP_LCD_TSCK_V  {GPIOA, GPIO_Pin_5, GPIO_Mode_Out_PP}
    #define BSP_LCD_TMOSI_V {GPIOA, GPIO_Pin_7, GPIO_Mode_Out_PP}
    #define BSP_LCD_TMISO_V {GPIOA, GPIO_Pin_6, GPIO_Mode_IPU}
    //Board specific LCD controller definitions
    #define BSP_LCD_NE         1  //FSMC chip select number (FSMC_NE1..FSMC_NE4) to which the LCD CS signal is connected
    #define BSP_LCD_RS_A       16 //FSMC address bus bit (FSMC_Axx) to which LCD RS signal is connected
    //#define LCD_REG01_CUSTOM   0  //Uncomment if the LCD image is mirrored and upside down
#endif
//-----------------------------------------------------------------------------------------


//Error checks
#if (BSP_LCD_NE != 1 && BSP_LCD_NE != 2 && BSP_LCD_NE != 3 && BSP_LCD_NE != 4)
    #error Incorrect BSP_LCD_NE value. Must be in range 1..4.
#endif

//Calculated values
#define BSP_LCD_FSMC_BCR ((volatile uint32_t*)(0xA0000000 + 8 * (BSP_LCD_NE - 1)))
#define BSP_LCD_FSMC_BTR ((volatile uint32_t*)(0xA0000000 + 4 + 8 * (BSP_LCD_NE - 1)))
#define BSP_LCD_FSMC_BWTR ((volatile uint32_t*)(0xA0000000 + 0x104 + 8 * (BSP_LCD_NE - 1)))

#if (BSP_LCD_NE == 1)
    #define BSP_LCD_IDX        ((volatile uint16_t*)0x60000000)  //LCD index address in memory space
    #define BSP_LCD_RAM        ((volatile uint16_t*)(0x60000000 + (1 << (BSP_LCD_RS_A + 1))))  //LCD RAM register address in memory space
#elif (BSP_LCD_NE == 2)
    #define BSP_LCD_IDX        ((volatile uint16_t*)0x64000000)  //LCD index address in memory space
    #define BSP_LCD_RAM        ((volatile uint16_t*)(0x64000000 + (1 << (BSP_LCD_RS_A + 1))))  //LCD RAM register address in memory space
#elif (BSP_LCD_NE == 3)
    #define BSP_LCD_IDX        ((volatile uint16_t*)0x68000000)  //LCD index address in memory space
    #define BSP_LCD_RAM        ((volatile uint16_t*)(0x68000000 + (1 << (BSP_LCD_RS_A + 1))))  //LCD RAM register address in memory space
#elif (BSP_LCD_NE == 4)
    #define BSP_LCD_IDX        ((volatile uint16_t*)0x6C000000)  //LCD index address in memory space
    #define BSP_LCD_RAM        ((volatile uint16_t*)(0x6C000000 + (1 << (BSP_LCD_RS_A + 1))))  //LCD RAM register address in memory space
#endif

#ifdef __cplusplus
}
#endif

#endif //BSP_H_INCLUDED

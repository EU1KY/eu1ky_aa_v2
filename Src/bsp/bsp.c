/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include "bsp.h"
#include "dbgprint.h"

static const BSP_GPIO bsp_gpio_cfg[] =
{//Entries must be in the same order as in BSP_GPIO_E
    BSP_LCD_WR_V,
    BSP_LCD_RD_V,
    BSP_LCD_DATA1_V,
    BSP_LCD_DATA2_V,
    BSP_LCD_RS_V,
    BSP_LCD_CS_V,
    BSP_LCD_RST_V,
    BSP_LCD_BL_V,
    BSP_LCD_BLPOL_V,
    BSP_LCD_TIRQ_V,
    BSP_LCD_TSS_V,
    BSP_LCD_TSCK_V,
    BSP_LCD_TMOSI_V,
    BSP_LCD_TMISO_V,
};

void BSP_Out_1(BSP_GPIO_E en)
{
    bsp_gpio_cfg[en].port->BSRR = bsp_gpio_cfg[en].pinmask;
}

void BSP_Out_0(BSP_GPIO_E en)
{
    bsp_gpio_cfg[en].port->BRR = bsp_gpio_cfg[en].pinmask;
}

uint32_t BSP_In(BSP_GPIO_E en)
{
    return (bsp_gpio_cfg[en].port->IDR & bsp_gpio_cfg[en].pinmask) ? 1 : 0;
}

void BSP_DelayUs(uint32_t us)
{
    //Calibrated for 72 MHz and code in flash
    asm volatile("           mov   r2, #14    \n"
                 "           mov   r1, #0     \n"
                 "2:         cmp   r1, r2     \n"
                 "           itt   lo         \n"
                 "           addlo r1, r1, #1 \n"
                 "           blo   2b         \n"
                 "           mov   r1, #8     \n"
                 "           mul   r2, %0, r1 \n"
                 "           mov   r1, #0     \n"
                 "1:         nop              \n"
                 "           cmp   r1, r2     \n"
                 "           itt   lo         \n"
                 "           addlo r1, r1, #1 \n"
                 "           blo   1b  \n"::"r"(us):"r1","r2");
}

//Calibrated for 72 MHz clock and code in flash
void BSP_DelayMs(uint32_t ms)
{
    uint32_t i;
    register const uint32_t count = 12000;
    for (i = 0; i < ms; i++)
    {
        asm volatile ("           mov   r1, #0     \n"
                      "1:         cmp   r1, %0     \n"
                      "           itt   lo         \n"
                      "           addlo r1, r1, #1 \n"
                      "           blo   1b  \n"::"r"(count):"r1");
    }
}

void BSP_Init(void)
{
    uint32_t i;
    GPIO_InitTypeDef gpioStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG,
                           ENABLE);
    RCC->AHBENR |= RCC_AHBENR_FSMCEN;

    //Initialize GPIO ports from bsp_gpio_cfg table
    for (i = 0; i < (sizeof(bsp_gpio_cfg)/sizeof(BSP_GPIO)); i++)
    {
        GPIO_StructInit(&gpioStruct);
        gpioStruct.GPIO_Speed = GPIO_Speed_10MHz;
        gpioStruct.GPIO_Pin = bsp_gpio_cfg[i].pinmask;
        gpioStruct.GPIO_Mode = bsp_gpio_cfg[i].mode;
        GPIO_Init(bsp_gpio_cfg[i].port, &gpioStruct);
    }

    //Initialize LCD FSMC
    //Configure FSMC interface to LCD

    //FSMC control register
    *BSP_LCD_FSMC_BCR = FSMC_BCR1_MWID_0 | FSMC_BCR1_MBKEN | FSMC_BCR1_WREN | FSMC_BCR1_EXTMOD;

    //FSMC read timings
    *BSP_LCD_FSMC_BTR = (0  << 0)   |  // Address setup phase duration (4 bits)
                        (40 << 8)   |  // Data-phase duration (8 bits) - width of /RD pulse in 12 ns intervals, minus 3
                                       // (worst case is SD1289 for which it must be at least 500 ns)
                        (15 << 16)  |  // Bus turn-around duration (4 bits)
                        (0  << 28);    // Access mode A;

    //FSMC write timings
    *BSP_LCD_FSMC_BWTR = (0  << 0)  |  // Address setup phase duration
                         (2  << 8)  |  // Data-phase duration
                         (5 << 16)  |  // Bus turn-around duration
                         (0  << 28);   // Access mode A;
    /*
    DBGPRINT("FSMC BCR: 0x%08X\n", (int)BSP_LCD_FSMC_BCR);
    DBGPRINT("FSMC BTR: 0x%08X\n", (int)BSP_LCD_FSMC_BTR);
    DBGPRINT("FSMC BWTR: 0x%08X\n", (int)BSP_LCD_FSMC_BWTR);
    DBGPRINT("LCD IDX: 0x%08X\n", (int)BSP_LCD_IDX);
    DBGPRINT("LCD RAM: 0x%08X\n", (int)BSP_LCD_RAM);
    */
}

/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include "touch.h"
#include "dbgprint.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "config.h"

static uint32_t TOUCH_rawx = 0;
static uint32_t TOUCH_rawy = 0;

unsigned int TOUCH_GetRawX(void)
{
    return TOUCH_rawx;
}

unsigned int TOUCH_GetRawY(void)
{
    return TOUCH_rawy;
}

//HW specific functions
static inline int _spi1_touchint_value(void)
{
    return BSP_In(BSP_LCD_TIRQ);
}

static inline int _spi1_miso_value(void)
{
    return BSP_In(BSP_LCD_TMISO);
}

static inline void _spi1_sck_low(void)
{
    BSP_Out_0(BSP_LCD_TSCK);
}

static inline void _spi1_sck_high(void)
{
    BSP_Out_1(BSP_LCD_TSCK);
}

static inline void _spi1_mosi_low(void)
{
    BSP_Out_0(BSP_LCD_TMOSI);
}

static inline void _spi1_mosi_high(void)
{
    BSP_Out_1(BSP_LCD_TMOSI);
}

static inline void _delayUs(uint32_t us)
{
    BSP_DelayUs(us);
}

static inline void _spi1_ntouchss_high(void)
{
    BSP_Out_1(BSP_LCD_TSS);
}

static inline void _spi1_ntouchss_low(void)
{
    BSP_Out_0(BSP_LCD_TSS);
}

/**
 Exchange byte over SPI interface.
 @note SPI is implemented by bitbanging because with APB1 clock of 72 MHz
 it is impossible to use hardware SPI1 on speeds below 285 kb/s. TSC2046
 requires speed of 125 kb/s or less.
 @param [in] byteTx Byte to transmit
 @return received byte
*/
static uint8_t SpiExchangeByte(uint8_t byteTx)
{
    uint8_t mask = 0x80;
    uint8_t rxByte = 0;
    while (mask)
    {
        (byteTx & mask) ? _spi1_mosi_high() : _spi1_mosi_low();
        _delayUs(5);
        if (_spi1_miso_value())
            rxByte |= mask;
        _spi1_sck_high();
        _delayUs(5);
        _spi1_sck_low();
        mask >>= 1;
    }
    return rxByte;
}


/** Poll touchscreen controller once. \n
    @note This function returns raw data measured by ADC. It is an overhead
    to convert it to coordinates here. It's better to do this after filtering
    several measurements.
    @param [in] coord the structure that will be filled with raw ADC data
    @return true if successful, false if touch is released.
*/
static int Poll2046Once(LCDPoint* pCoord)
{
    //16-bits, DFR mode, measure Y, X.
    static const uint8_t txBytes[5] = {0x92, 0x00, 0xD2, 0x00, 0x00};
    uint8_t rxBytes[5];

    uint32_t i;

    if (0 == pCoord)
        return 0;

    if (_spi1_touchint_value())
        return 0;

    //Chip select
    _spi1_ntouchss_low();
    _delayUs(5);

    //Exchange bytes
    for (i = 0; i < 5; ++i)
    {
        rxBytes[i] = SpiExchangeByte(txBytes[i]);
    }
    //Deselect
    _spi1_ntouchss_high();
    _delayUs(2);

    //Touch has been released before ADC conversion is finished:
    //the result can be invalid.
    if (_spi1_touchint_value())
        return 0;

    //Now extract raw ADC values
    uint16_t rawY =  (((uint16_t)rxBytes[1]) << 5) | (((uint16_t)rxBytes[2]) >> 3);
    uint16_t rawX =  (((uint16_t)rxBytes[3]) << 5) | (((uint16_t)rxBytes[4]) >> 3);
    if (TOUCH_cal->reversed_xy)
    {
        pCoord->x = rawX;
        pCoord->y = rawY;
    }
    else
    {
        pCoord->y = rawX;
        pCoord->x = rawY;
    }
    return 1;
}

int TOUCH_IsPressed(void)
{
    return !_spi1_touchint_value();
}


/** Poll touchscreen controller, filter data and calculate real coordinates.
    @param [in] coord the structure that will be filled with raw ADC data
    @return true if successful, false if touch is released.
*/
int TOUCH_Poll(LCDPoint* pCoord)
{
    LCDPoint polls[5];
    uint16_t max = 0;
    uint16_t min = 4096;
    int imin = 0;
    int imax = 0;
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t resX = 0;
    uint16_t resY = 0;

    if (0 == pCoord)
        return 0;

    //Poll controller 5 times
    int i;
    for (i = 0; i < 5; ++i)
    {
        if (0 == Poll2046Once(&polls[i]))
            return 0; //touch has been released
        _delayUs(500);
    }

    //Now find max and min x to exclude
    for (i = 0; i < 5; ++i)
    {
        if (polls[i].x < min) { min = polls[i].x; imin = i; }
        if (polls[i].x > max) { max = polls[i].x; imax = i; }
    }
    if (imin == imax)
    {//no need to calculate median
        x = polls[imin].x;
    }
    else
    {
        x = 0;
        for (i = 0; i < 5; ++i)
        {
            if (i == imin || i == imax) continue; //Skip
            x += polls[i].x;
        }
        x /= 3; //Median value of x
    }

    //Repeat the same for y
    max = 0;
    min = 4096;
    for (i = 0; i < 5; ++i)
    {
        if (polls[i].y < min) { min = polls[i].y; imin = i; }
        if (polls[i].y > max) { max = polls[i].y; imax = i; }
    }

    if (imin == imax)
    {//no need to calculate median
        y = polls[imin].y;
    }
    else
    {
        y = 0;
        for (i = 0; i < 5; ++i)
        {
            if (i == imin || i == imax) continue;
            y += polls[i].y;
        }
        y /= 3; //Median value of y
    }

    TOUCH_rawx = x; //Store for calibration purposes
    TOUCH_rawy = y;

    if (x > TOUCH_cal->xmax) x = TOUCH_cal->xmax;
    else if (x < TOUCH_cal->xmin) x = TOUCH_cal->xmin;
    x -= TOUCH_cal->xmin;
    if (y > TOUCH_cal->ymax) y = TOUCH_cal->ymax;
    else if (y < TOUCH_cal->ymin) y = TOUCH_cal->ymin;
    y -= TOUCH_cal->ymin;

    //Now convert to real coordinates on screen
    resX = (((uint32_t)x) * LCD_GetWidth()) / TOUCH_cal->xbw;
    resY = (((uint32_t)y) * LCD_GetHeight()) / TOUCH_cal->ybw;
    if (TOUCH_cal->invert_x)
        resX = LCD_GetWidth() - resX;
    if (TOUCH_cal->invert_y)
        resY = LCD_GetHeight() - resY;

    if (resX >= LCD_GetWidth())
        resX = LCD_GetWidth() - 1;
    if (resY >= LCD_GetHeight())
        resY = LCD_GetHeight() - 1;
    pCoord->x = resX;
    pCoord->y = resY;
    DBGPRINT("TOUCH: X=%d Y=%d\n", (int)resX, (int)resY);
    return 1;
}

void TOUCH_Init(void)
{
    //BSP_Init() should have been called before entering this function
    _spi1_sck_low();
    _spi1_ntouchss_high();
}

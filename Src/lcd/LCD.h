/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef LCD_H_INCLUDED
#define LCD_H_INCLUDED

#include <stdint.h>

#include "config.h" // for LCD_ORIENTATION_HORIZONTAL / LCD_ORIENTATION_VERTICAL,
                    // LCD_BACKLIGHT_INVERTED and LCD_FSMC_DIVIDE_RATIO

#ifdef __cplusplus
extern "C" {
#endif

///LCD point descriptor
typedef struct __attribute__((packed))
{
    uint16_t x;
    uint16_t y;
} LCDPoint;


///LCD color type (contains one og 65536 colors in 565 format)
typedef uint16_t LCDColor;

///Convert 24-bit RGB color to 565 format (65536 colors) with macro
#define LCD_RGB(r, g, b) ((LCDColor)( \
                             ((( ((uint16_t)(r)) >> 3) & 0x1F) << 11) |  \
                             ((( ((uint16_t)(g)) >> 2) & 0x3F) << 5) |   \
                             ((  ((uint16_t)(b)) >> 3) & 0x1F)           \
                         ))

///Some useful color definitions
enum
{
    LCD_BLACK = LCD_RGB(0, 0, 0),
    LCD_GRAY = LCD_RGB(127, 127, 127),
    LCD_RED = LCD_RGB(255, 0, 0),
    LCD_GREEN = LCD_RGB(0, 255, 0),
    LCD_BLUE = LCD_RGB(0, 0, 255),
    LCD_YELLOW = LCD_RGB(255, 255, 0),
    LCD_PURPLE = LCD_RGB(255, 0, 255),
    LCD_CYAN = LCD_RGB(0, 255, 255),
    LCD_WHITE = LCD_RGB(255, 255, 255)
};

///Initialize hardware, turn on and fill display with black
void LCD_Init(void);

///Make LCDPoint from x and y coordinates
LCDPoint LCD_MakePoint(int x, int y);

///Make LCDColor from R, G and B components with function. See also
///LCD_RGB macro that does the same at compile time.
LCDColor LCD_MakeRGB(uint8_t r, uint8_t g, uint8_t b);

///Sets pixel at given point to given color
void LCD_SetPixel(LCDPoint p, LCDColor color);

///Fill rectangle with given corner points with given color
void LCD_FillRect(LCDPoint p1, LCDPoint p2, LCDColor color);

///Fill the entire display with given color
void LCD_FillAll(LCDColor c);

///Draw lines forming a rectangle with given corner points with given color
void LCD_Rectangle(LCDPoint a, LCDPoint b, LCDColor c);


void LCD_Circle(LCDPoint center, int16_t r, LCDColor color);

void LCD_FillCircle(LCDPoint center, int16_t r, LCDColor color);

///Draw line between given points with given color
void LCD_Line(LCDPoint p1, LCDPoint p2, LCDColor c);

///Turn on LCD and backlight
void LCD_TurnOn(void);

///Turn off backlight and switch LCD to power saving mode (but draving to its
///memory remains available)
void LCD_TurnOff(void);

///Turn on LCD backlight
void LCD_BacklightOn(void);

///Turn off LCD backlight
void LCD_BacklightOff(void);

///Programmatically delay for given number of milliseconds. Heavily loads CPU,
///Can be used when no timers are on in the system.
void LCD_DelayMs(uint32_t ms);

///Draw bitmap file contents at given origin point
void LCD_DrawBitmap(LCDPoint origin, const uint8_t *bmpData, uint32_t bmpDataSize);

///Invert color of display pixel
void LCD_InvertPixel(LCDPoint p);

uint16_t LCD_GetWidth(void);

uint16_t LCD_GetHeight(void);

#ifdef __cplusplus
}
#endif

#endif //LCD_H_INCLUDED

/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "bkup.h"
#include "stm32f10x.h"
#include "touch.h"
#include "LCD.h"
#include "font.h"

void Sleep(uint32_t nms);

static void DrawPoint(uint16_t x, uint16_t y, LCDColor clr)
{
    LCD_Circle(LCD_MakePoint(x,y), 7, clr);
    LCD_Line(LCD_MakePoint(x-7, y), LCD_MakePoint(x+7, y), clr);
    LCD_Line(LCD_MakePoint(x, y-7), LCD_MakePoint(x, y+7), clr);
}

static void SaveTSCALParams(uint32_t *pData)
{
    uint32_t address = (uint32_t)TOUCH_cal;
    FLASH_UnlockBank1();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    FLASH_ErasePage(address);
    uint32_t i;
    for (i = 0; i < (sizeof(struct TSCAL_PARAMS) / sizeof(uint32_t)); i++)
    {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
        FLASH_ProgramWord(address + i * 4, pData[i]);
    }
    FLASH_LockBank1();
}

int TSCAL_IsCalibrated(void)
{
    if (TOUCH_cal->xmin < 4096 && TOUCH_cal->xmin != 0)
        return 1;
    return 0;
}

void TSCAL_Proc2(void)
{
    LCDPoint coord;

    for(;;)
    {
        struct TSCAL_PARAMS params = {0};
        SaveTSCALParams((uint32_t*)&params);

        while(TOUCH_IsPressed());
        uint16_t x[4] = {0};
        uint16_t y[4] = {0};

        LCD_FillAll(LCD_BLACK);
        FONT_SetAttributes(FONT_FRAN, LCD_WHITE, LCD_BLACK);

        FONT_Printf(10, 110, "Tap top left point");
        //Top left
        DrawPoint(0, 0, LCD_WHITE);
        while(!TOUCH_Poll(&coord));
        x[0] = TOUCH_GetRawX();
        y[0] = TOUCH_GetRawY();
        DrawPoint(0, 0, LCD_GREEN);
        while(TOUCH_IsPressed());

        //Top right
        FONT_Printf(10, 110, "Tap top right point     ");
        DrawPoint(319, 0, LCD_WHITE);
        while(!TOUCH_Poll(&coord));
        x[1] = TOUCH_GetRawX();
        y[1] = TOUCH_GetRawY();
        DrawPoint(319, 0, LCD_GREEN);
        while(TOUCH_IsPressed());

        //Lower right
        FONT_Printf(10, 110, "Tap lower right point    ");
        DrawPoint(319, 239, LCD_WHITE);
        while(!TOUCH_Poll(&coord));
        x[2] = TOUCH_GetRawX();
        y[2] = TOUCH_GetRawY();
        DrawPoint(319, 239, LCD_GREEN);
        while(TOUCH_IsPressed());

        //Lower left
        FONT_Printf(10, 110, "Tap lower left point"     );
        DrawPoint(0, 239, LCD_WHITE);
        while(!TOUCH_Poll(&coord));
        x[3] = TOUCH_GetRawX();
        y[3] = TOUCH_GetRawY();
        DrawPoint(0, 239, LCD_GREEN);
        while(TOUCH_IsPressed());

        //Print results
        FONT_Printf(0, 110, "x: %d %d %d %d           ", x[0], x[1], x[2], x[3]);
        FONT_Printf(0, 130, "y: %d %d %d %d ", y[0], y[1], y[2], y[3]);

        //Calculate calibration parameters
        int i;

        if ((x[0] < 1500 && x[1] < 1500 && x[2] >= 1500 && x[3] >= 1500) ||
                (x[0] >= 1500 && x[1] >= 1500 && x[2] < 1500 && x[3] < 1500))
        {
            // X and Y are reversed
            params.reversed_xy = 1;
            uint32_t tmp;
            for (i = 0; i < 4; i++)
            {
                tmp = x[i];
                x[i] = y[i];
                y[i] = tmp;
            }
        }

        if (x[1] < x[0])
            params.invert_x = 1;
        if (y[3] < y[0])
            params.invert_y = 1;

        params.xmin = 0xFFFFFFFFul;
        params.xmax = 0;
        params.ymin = 0xFFFFFFFFul;
        params.ymax = 0;
        for (i = 0; i < 4; i++)
        {
            if (x[i] < params.xmin)
                params.xmin = x[i];
            if (x[i] > params.xmax)
                params.xmax = x[i];
            if (y[i] < params.ymin)
                params.ymin = y[i];
            if (y[i] > params.ymax)
                params.ymax = y[i];
        }
        params.xbw = params.xmax - params.xmin;
        params.ybw = params.ymax - params.ymin;

        SaveTSCALParams((uint32_t*)&params);
        FONT_Printf(0, 150, "Saved! Now try to draw to see the touchscreen works OK.");
        FONT_Printf(0, 170, "It must not be very precise.");
        FONT_Printf(0, 190, "Power off the device to exit.");

        while(1)
        {
            if (TOUCH_Poll(&coord))
                LCD_SetPixel(coord, LCD_PURPLE);
        }
    }
}

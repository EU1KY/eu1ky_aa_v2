/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

//==============================================
// EU1KY antenna analyzer project
// Main file
//==============================================

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x.h"
#include "font.h"
#include "LCD.h"
#include "touch.h"
#include "dsp.h"
#include "btusart.h"
#include "protocol.h"
#include "measurement.h"
#include "panvswr2.h"
#include "generator.h"
#include "bkup.h"
#include "TouchScreenCalibration.h"
#include "dbgprint.h"
#include "config.h"
#include "si5351.h"
#include "gen.h"
#include "osl.h"
#include "configwnd.h"
#include "bsp.h"

/*********************************************************************
 *
 *  main()
 *
 *********************************************************************/
asm(".global _printf_float"); //To enable floating point support in NewLib-nano formatted I/O functions.

//===========================================================
// Functionality to get millisecond ticks
static volatile uint32_t _msCounter = 0;

void SysTick_Handler()
{
    ++_msCounter;
}

uint32_t GetTickCount()
{
    return _msCounter;
}

void Sleep(uint32_t nms)
{
    uint32_t tStart = GetTickCount();
    while(GetTickCount() - tStart < nms);
}
//===========================================================

static void Detect_HW(void)
{
    uint32_t fail_count = 0;
    uint8_t si_reg_0 = si5351_read_device_status();

    if (0 == si_reg_0 || 0xFF == si_reg_0)
    {
        DBGPRINT("Si5351a not detected\n");
        FONT_Write(FONT_FRAN, LCD_RED, LCD_BLACK, 10, 150, "Si5351a not detected");
        ++fail_count;
    }
    else
    {
        FONT_Write(FONT_FRAN, LCD_GREEN, LCD_BLACK, 10, 150, "Si5351a detected");
    }
    DSP_Measure(10000000ul, 0, 0, 3);
    GEN_SetMeasurementFreq(0);

    if (DSP_MeasuredMagImv() < 100. && DSP_MeasuredMagQmv() < 100.)
    {
        DBGPRINT("RF frontend not detected\n");
        FONT_Write(FONT_FRAN, LCD_RED, LCD_BLACK, 10, 170, "No or low signal from RF frontend");
        ++fail_count;
    }
    else
    {
        FONT_Write(FONT_FRAN, LCD_GREEN, LCD_BLACK, 10, 170, "RF frontend detected");
    }
    if (fail_count)
        Sleep(2000);
}

int main()
{
    SysTick_Config(SystemCoreClock / 1000);

    BSP_Init();
    BKUP_Initialize();
    LCD_Init();
    TOUCH_Init();
    si5351_init();
    DSP_Init();
    BTUSART_Init(); //Used only in remote control window proc

    setvbuf(stdin, NULL, _IONBF, 0); //turn off STDIN buffering
    setvbuf(stdout, NULL, _IONBF, 0); //turn off STDOUT buffering
    DBGPRINT("EU1KY antenna analyzer v " VER "\n");
    Sleep(100);

    // Temporary code to change HC-06 Bluetooth module name.
    // Connect Wakeup to Vcc on BT module then power cycle in order to enter
    // AT mode and change BT module name. Like here:
    //BTUSART_PutString("AT+NAMEAntennaAnalyzer\n");
    //for(;;) Sleep(10);

    if (!TSCAL_IsCalibrated() || TOUCH_IsPressed())
    { //If started up with touchscreen pressed, or touchscreen is not calibrated -
      //enter touchscreen calibration window
        TSCAL_Proc2();
    }

    LCD_FillAll(LCD_BLACK);
    FONT_Write(FONT_CONSBIG, LCD_GREEN, LCD_BLACK, 100, 60, "E U 1 K Y");
    FONT_Write(FONT_FRANBIG, LCD_CYAN, LCD_BLACK, 27, 100, "antenna analyzer v " VER);

    Detect_HW();
    Sleep(1000);

    DBGPRINT("MAIN: Config window\n");
    CONFIG_Proc();

    for(;;)
    {
        //main loop
        DBGPRINT("MAIN: Panoramic window\n");
        PANVSWR2_Proc();

        DBGPRINT("MAIN: Measurement window\n");
        MEASUREMENT_Proc();

        DBGPRINT("MAIN: Generator window\n");
        GENERATOR_Proc();

        DBGPRINT("MAIN: Remote control window\n");
        PROTOCOL_Proc();
    }// for(;;) - main loop

    return 0;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    DBGPRINT("ASSERTION FAILED at %s line %u\n", file, (unsigned int)line);
    for(;;);
}
#endif

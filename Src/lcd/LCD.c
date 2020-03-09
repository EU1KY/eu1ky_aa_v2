/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include "LCD.h"
#include "bsp.h"
#include "libnsbmp.h"
#include "dbgprint.h"

//LCD device codes from register 0
#define LCD_DEV_9325 0x9325
#define LCD_DEV_9320 0x9320
#define LCD_DEV_1289 0x8989
#define LCD_DEV_5408 0x5408

static uint16_t deviceCode = LCD_DEV_9325;

//Helper to set gamma register values
#define GREGVAL(byte1, byte2) (uint16_t)(((byte1) << 8) | ((byte2) & 0xFF))
//Helper for bit mask
#define b(n) (1<<(n))
//IL9325 bits
enum
{
    //Reg 01h Driver Output control
    SS=b(8), SM=b(10),
    //Reg 02h LCD Driving control
    EOR=b(8), BC0=b(9),
    //Reg 03h Entry mode
    AM=b(3), ID0=b(4), ID1=b(5), ORG=b(7), HWM=b(9), BGR=b(12),
    DFM=b(14), TRI=b(15),
    //Reg 04h Resize control
    RSZ0=b(0), RSZ1=b(1), RCH0=b(4), RCH1=b(5), RCV0=b(8), RCV1=b(9),
    //Reg 07h Display control 1
    D0=b(0), D1=b(1), CL=b(3), DTE=b(4), GON=b(5), BASEE=b(8),
    PTDE0=b(12), PTDE1=b(13),
    //Reg 08h Display Control 2
    BP0=b(0), BP1=b(1), BP2=b(2), BP3=b(3),
    FP0=b(8), FP1=b(9), FP2=b(10), FP3=b(11),
    //Reg 09h Display Control 3
    ISC0=b(0), ISC1=b(1), ISC2=b(2), ISC3=b(3),
    PTG0=b(4), PTG1=b(5),
    PTS0=b(8), PTS1=b(9), PTS2=b(10),
    //Reg 0Ah Display Control 4
    FMI0=b(0), FMI1=b(1), FMI2=b(3), FMARKOE=b(3),
    //Reg 0Ch RGB Display Interface Control 1
    ENC2=b(14), ENC1=b(13), ENC0=b(12), RM=b(8), DM1=b(5), DM0=b(4),
    RIM1=b(1), RIM0=b(0),
    //Reg 0Dh Frame Maker Position
    FMP8=b(8), FMP7=b(7), FMP6=b(6), FMP5=b(5), FMP4=b(4), FMP3=b(3),
    FMP2=b(2), FMP1=b(1), FMP0=b(0),
    //Reg 0Fh RGB Display Interface Control 2
    EPL=b(0), DPL=b(1), HSPL=b(3), VSPL=b(4),
    //Reg 10h Power Control 1
    SAP=b(12), BT2=b(10), BT1=b(9), BT0=b(8), APE=b(7),
    AP2=b(6), AP1=b(5), AP0=b(4), DSTB=b(2), SLP=b(1), STB=b(0),
    //Reg 11h Power Control 2
    DC12=b(10), DC11=b(9), DC10=b(8), DC02=b(6), DC01=b(5), DC00=b(4),
    VC2=b(2), VC1=b(1), VC0=b(0),
    //Reg 12h Power Control 3
    VCIRE=b(7), PON=b(4), VRH3=b(3), VRH2=b(2), VRH1=b(1), VRH0=b(0),
    //Reg 13h Power Control 4
    VDV4=b(12), VDV3=b(11), VDV2=b(10), VDV1=b(9), VDV0=b(8),
    //Reg 20h Horizontal GRAM Address Set (0..239)
    //Reg 21h Vertical GRAM Address Set (0...319)
    //Reg 22h Write Data to GRAM
    //Reg 29h Power Control 7
    VCM5=b(5), VCM4=b(4), VCM3=b(3), VCM2=b(2), VCM1=b(1), VCM0=b(0),
    //Reg 2Bh Frame Rate and Color Controls
    FRS3=b(3), FRS2=b(2), FRS1=b(1), FRS0=b(0),
    //Reg 30h Gamma Control 1:  GREGVAL(KP1, KP0)
    //Reg 31h Gamma Control 2:  GREGVAL(KP3, KP2)
    //Reg 32h Gamma Control 3:  GREGVAL(KP5, KP4)
    //Reg 35h Gamma Control 4:  GREGVAL(RP1, RP0)
    //Reg 36h Gamma Control 5:  GREGVAL(VRP1, VRP0)
    //Reg 37h Gamma Control 6:  GREGVAL(KN1, KN0)
    //Reg 38h Gamma Control 7:  GREGVAL(KN3, KN2)
    //Reg 39h Gamma Control 8:  GREGVAL(KN5, KN4)
    //Reg 3Ch Gamma Control 9:  GREGVAL(RN1, RN0)
    //Reg 3Dh Gamma Control 10: GREGVAL(VRN1, VRN0)
    //Reg 50h Horizontal Address Start Position (0...239)
    //Reg 51h Horizontal Address End Position (0...239)
    //Reg 52h Vertical Address Start Position (0...319)
    //Reg 53h Vertical Address End Position (0...319)
    //Reg 60h Driver Output Control 2
    GS=b(15), NL5=b(13), NL4=b(12), NL3=b(11), NL2=b(10), NL1=b(9), NL0=b(8),
    SCN5=b(5), SCN4=b(4), SCN3=b(3), SCN2=b(2), SCN1=b(1), SCN0=b(0),
    //Reg 61h Base Image Display Control
    NDL=b(2), VLE=b(1), REV=b(0),
    //Reg 6Ah Vertical Scroll Control (0...319)
    //Reg 80h Partial Image 1 Display Position  (9 bits) PTDP00..08
    //Reg 81h Partial Image 1 Area (Start Line) (9 bits) PTSA00..08
    //Reg 82h Partial Image 1 Area (End Line) (9 bits) PTEA00..08
    //Reg 83h Partial Image 2 Display Position (9 bits) PTDP10..18
    //Reg 84h Partial Image 2 Area (Start Line) (9 bits) PTSA10..18
    //Reg 85h Partial Image 2 Area (End Line) (9 bits) PTEA10..18
    //Reg 90h Panel Interface Control 1
    DIVI1=b(9), DIVI00=b(8), RTNI3=b(3), RTNI2=b(2), RTNI1=b(1), RTNI0=b(0),
    //Reg 92h Panel Interface Control 2
    NOWI2=b(10), NOWI1=b(9), NOWI0=b(8),
    //Reg 95h Panel Interface Control 4
    DIVE1=b(9), DIVE0=b(8),
    RTNE5=b(5), RTNE4=b(4), RTNE3=b(3), RTNE2=b(2), RTNE1=b(1), RTNE0=b(0),
    //Reg A1h OTP VCM Programming Control OTP_PGM_EN + 6 bits (VCM_OTP0..5)
    OTP_PGM_EN=b(11),
    //Reg A2h OTP VCM Status and Enable
    PGM_CNT1=b(15), PGM_CNT0=b(14),
    VCM_D5=b(13), VCM_D4=b(12), VCM_D3=b(11), VCM_D2=b(10),
    VCM_D1=b(9), VCM_D0=b(8),
    VCM_EN=b(0),
    //Reg A5h OTP Programming ID Key (16 bits)
}; //enum


//==============================================================================
//                  Module's static functions
//==============================================================================
static inline void _DispResetOn(void)
{
    BSP_Out_0(BSP_LCD_RST);
}

static inline void _DispResetOff(void)
{
    BSP_Out_1(BSP_LCD_RST);
}

static inline void _DispBacklightOn(void)
{
    if (BSP_In(BSP_LCD_BLPOL))
    {
        BSP_Out_0(BSP_LCD_BL);
    }
    else
    {
        BSP_Out_1(BSP_LCD_BL);
    }
}

static inline void _DispBacklightOff(void)
{
    if (BSP_In(BSP_LCD_BLPOL)) //LCD_BACKLIGHT_INVERTED
    {
        BSP_Out_1(BSP_LCD_BL);
    }
    else
    {
        BSP_Out_0(BSP_LCD_BL);
    }
}

static inline void _writeIdx(uint8_t reg)
{
    *BSP_LCD_IDX = reg;
}

static inline void _writeRam(uint16_t data)
{
    *BSP_LCD_RAM = data;
}

static inline uint16_t _readRam()
{
    return *BSP_LCD_RAM;
}

static inline void _writeReg(uint8_t reg, uint16_t data)
{
    *BSP_LCD_IDX = reg;
    *BSP_LCD_RAM = data;
}

static inline uint16_t _readReg(uint8_t reg)
{
    *BSP_LCD_IDX = reg;
    return *BSP_LCD_RAM;
}

static inline uint16_t _min(uint16_t a, uint16_t b)
{
    return a > b ? b : a;
}

static inline uint16_t _max(uint16_t a, uint16_t b)
{
    return a > b ? a : b;
}

static inline int _abs(int a)
{
    return (a < 0) ? -a : a;
}

static inline void _setCursor(LCDPoint p)
{
    assert_param(p.x < LCD_GetWidth());
    assert_param(p.y < LCD_GetHeight());
    if (deviceCode == LCD_DEV_1289)
    {
        _writeReg(0x4E, p.y);     //GDDRAM X position
        _writeReg(0x4F, LCD_GetWidth() - 1 - p.x); //GDDRAM Y position
    }
    else
    {
        _writeReg(0x20, p.y);
        _writeReg(0x21, LCD_GetWidth() - 1 - p.x);
    }
}

static inline void _LimitedWindow(LCDPoint p1, LCDPoint p2)
{
    assert_param(p2.x >= p1.x);
    assert_param(p2.y >= p1.y);
    assert_param(p2.x < LCD_GetWidth());
    assert_param(p2.y < LCD_GetHeight());
    if (deviceCode == LCD_DEV_1289)
    {
#ifdef LCD_ENTRY_MODE
        _writeReg(0x11, LCD_ENTRY_MODE); //Entry mode defined in BSP
#else
        _writeReg(0x11, 0x6058); //ID = 01 AM = 1  - Entry mode
#endif
        _writeReg(0x44, p1.y | (p2.y << 8) ); //Horizontal RAM address position
        _writeReg(0x45, LCD_GetWidth() - 1 - p2.x);  //Vertical RAM address position
        _writeReg(0x46, LCD_GetWidth() - 1 - p1.x);  //Vertical RAM address end position
    }
    else
    {
#ifdef LCD_ENTRY_MODE
        _writeReg(0x03, LCD_ENTRY_MODE); //Entry mode defined in BSP
#else
        _writeReg(0x03, (BGR | ID0));//GRAM increment left-to-right first, then up to down
#endif
        _writeReg(0x50, p1.y);
        _writeReg(0x51, p2.y);
        _writeReg(0x52, LCD_GetWidth() - 1 - p2.x);
        _writeReg(0x53, LCD_GetWidth() - 1 - p1.x);
    }
    _setCursor(p1);
}

uint16_t LCD_GetWidth(void)
{
    return 320;
}

uint16_t LCD_GetHeight(void)
{
    return 240;
}

void LCD_BacklightOn(void)
{
    _DispBacklightOn();
}

void LCD_BacklightOff(void)
{
    _DispBacklightOff();
}

static void LCD_ILI9325_Init(void)
{
    _writeReg(0x00, 0x0001); // Start Oscillation
    LCD_DelayMs(10);
#if defined LCD_REG01_CUSTOM
    _writeReg(0x01, LCD_REG01_CUSTOM);
#else
    _writeReg(0x01, SS); // SS=1, SM=0
#endif
    _writeReg(0x02, BC0 | EOR); // BC0=1 EOR=1

    _writeReg(0x03, BGR | ID1 | ID0 | AM); //Entry mode
    _writeReg(0x04, 0x0000); // Resize register
    _writeReg(0x08, 0x0202); // Front and Back Porch (must be >= 2 lines)
    _writeReg(0x09, 0x0000); // Normal scan, Scan cycle=0
    _writeReg(0x0A, 0x0000); // Output of FMARK signal is disabled
    _writeReg(0x0C, RIM0);   // System interface, 16 bits, internal clock
    _writeReg(0x0D, 0x0000); // Frame marker Position
    _writeReg(0x0F, 0x0000); // System interface polarity (CLK, EN)

    //Power-up sequence
    _writeReg(0x10, 0x0000);
    _writeReg(0x11, VC0|VC1|VC2); //Vci1 = 1.0 * Vci
    _writeReg(0x12, 0x0000);      //External reference
    _writeReg(0x13, 0x0000);      //VCOM = 0.70 * VREG1OUT
    LCD_DelayMs(200);             //Let the voltage stabilize

    _writeReg(0x10, SAP | BT2 | BT1 | APE | AP0); //Enable power supply circuits
    _writeReg(0x11, VC0 | DC01 | DC11); //Step-up circuits parameters
    LCD_DelayMs(50);                 // Let it stabilize
    _writeReg(0x12, PON | VRH3 | VRH2); //Set grayscale level
    LCD_DelayMs(50);                 // Let it stabilize
    _writeReg(0x13, VDV4 | VDV1 | VDV0); //Set Vcom voltage
    _writeReg(0x29, VCM4 | VCM3 | VCM2); // Set VcomH voltage
    _writeReg(0x2B, 0x000D); //Set frame rate 93Hz

    //Gamma
    //Setting all values for the linear curve, except RN1 (to increase contrast
    //in the darkest area)
    _writeReg(0x30, GREGVAL(3,  3));  //KP1[3] KP0[3]   Fine adjustment positive: grayscale 8, grayscale 1
    _writeReg(0x31, GREGVAL(3,  3));  //KP3[3] KP2[3]   Fine adjustment positive: grayscale 43, grayscale 20
    _writeReg(0x32, GREGVAL(3,  3));  //KP5[3] KP4[3]   Fine adjustment positive: grayscale 62, grayscale 55
    _writeReg(0x35, GREGVAL(1,  1));  //RP1[3] RP0[3]   Gradient adjustment variable resistors, positive
    _writeReg(0x36, GREGVAL(15, 15)); //VRP1[5] VRP0[5] Amplitude adjustment variable resistors, positive
    _writeReg(0x37, GREGVAL(3,  3));  //KN1[3] KN0[3]   Fine adjustment negative: grayscale 8, grayscale 1
    _writeReg(0x38, GREGVAL(3,  3));  //KN3[3] KN2[3]   Fine adjustment negative: grayscale 43, grayscale 20
    _writeReg(0x39, GREGVAL(3,  3));  //KN5[3] KN4[3]   Fine adjustment negative: grayscale 62, grayscale 55
    _writeReg(0x3C, GREGVAL(7,  1));  //RN1[3] RN0[3]   Gradient adjustment variable resistors, negative
    _writeReg(0x3D, GREGVAL(15, 15)); //VRN1[5] VRN0[5] Amplitude adjustment variable resistors, negative

    //GRAM
    _writeReg(0x50, 0x0000); // Horizontal GRAM Start Address
    _writeReg(0x51, 0x00EF); // Horizontal GRAM End Address
    _writeReg(0x52, 0x0000); // Vertical GRAM Start Address
    _writeReg(0x53, 0x013F); // Vertical GRAM Start Address
    _writeReg(0x60, 0xA700); // Gate Scan Line
    _writeReg(0x61, 0x0001); // NDL,VLE, REV
    _writeReg(0x6A, 0x0000); // Scrolling line

    //Partial Display
    _writeReg(0x80, 0x0000);
    _writeReg(0x81, 0x0000);
    _writeReg(0x82, 0x0000);
    _writeReg(0x83, 0x0000);
    _writeReg(0x84, 0x0000);
    _writeReg(0x85, 0x0000);

    //Panel Control
    _writeReg(0x90, 0x0010);
    _writeReg(0x92, 0x0000);
    _writeReg(0x93, 0x0003);
    _writeReg(0x95, 0x0110);
    _writeReg(0x97, 0x0000);
    _writeReg(0x98, 0x0000);

    // Display ON
    _writeReg(0x07, D0 | D1 |DTE | GON | BASEE);
}

static void LCD_ILI9320_Init(void)
{
    _writeReg(0x00, 0x0001);
    LCD_DelayMs(10);

    _writeReg(0xE5, 0x8000);
    _writeReg(0xE3, 0x3008);
    _writeReg(0xE7, 0x0012);
    _writeReg(0xEF, 0x1231);
#if defined LCD_REG01_CUSTOM
    _writeReg(0x01, LCD_REG01_CUSTOM);
#else
    _writeReg(0x01, SS); // SS=1, SM=0
#endif
    _writeReg(0x02, 0x0400 | BC0 | EOR); //LCD driving control

    _writeReg(0x03, BGR | ID1 | ID0 | AM); //Entry mode
    _writeReg(0x04, 0x0000); // Resize register
    _writeReg(0x08, 0x0202); // Front and Back Porch (must be >= 2 lines)
    _writeReg(0x09, 0x0000); // Normal scan, Scan cycle=0
    _writeReg(0x0A, 0x0000); // Output of FMARK signal is disabled
    _writeReg(0x0C, RIM0);   // System interface, 16 bits, internal clock
    _writeReg(0x0D, 0x0000); // Frame marker Position
    _writeReg(0x0F, 0x0000); // System interface polarity (CLK, EN)

    // Power Control
    _writeReg(0x07, 0x0101); //power control 1 BT, AP
    _writeReg(0x10, 0x0000);
    _writeReg(0x11, VC0|VC1|VC2); //Vci1 = 1.0 * Vci
    _writeReg(0x12, 0x0000); //TODO: this one differs from 9325. Consider VCMR bit
    _writeReg(0x13, 0x0000); //VCOM = 0.70 * VREG1OUT
    LCD_DelayMs(200); //Let the voltage stabilize

    _writeReg(0x10, 0x16B0); //power control 1 BT,AP - differs from 9325
    _writeReg(0x11, 0x0037); //power control 2 DC,VC - differs from 9325
    LCD_DelayMs(50);         // Let it stabilize
    _writeReg(0x12, 0x013E); //power control 3 VRH - differs from 9325
    LCD_DelayMs(50);
    _writeReg(0x13, 0x1A00); //power control 4 vcom amplitude
    _writeReg(0x29, 0x000F); //power control 7 VCOMH
    _writeReg(0x2B, 0x0010); //Setting 90Hz frame rate. Differs from 9325!
    LCD_DelayMs(50);

    // GAMMA Control (values were taken from some code - maybe adjustment needed)
    _writeReg(0x30, 0x0007);
    _writeReg(0x31, 0x0403);
    _writeReg(0x32, 0x0404);
    _writeReg(0x35, 0x0002);
    _writeReg(0x36, 0x0707);
    _writeReg(0x37, 0x0606);
    _writeReg(0x38, 0x0106);
    _writeReg(0x39, 0x0007);
    _writeReg(0x3c, 0x0700);
    _writeReg(0x3d, 0x0707);

    //GRAM
    _writeReg(0x50, 0x0000); //Horizontal Address Start Position
    _writeReg(0x51, 0x00EF); //Horizontal Address end Position (239)
    _writeReg(0x52, 0x0000); //Vertical Address Start Position
    _writeReg(0x53, 0x013F); //Vertical Address end Position (319)
    _writeReg(0x20, 0x0000); //Horizontal GRAM Address Set
    _writeReg(0x21, 0x0000); //Vertical GRAM Address Set

    _writeReg(0x60, 0xA700); // Gate Scan Line
    _writeReg(0x61, 0x0001); // NDL,VLE, REV
    _writeReg(0x6A, 0x0000); // Scrolling line

    //Partial Display
    _writeReg(0x80, 0x0000);
    _writeReg(0x81, 0x0000);
    _writeReg(0x82, 0x0000);
    _writeReg(0x83, 0x0000);
    _writeReg(0x84, 0x0000);
    _writeReg(0x85, 0x0000);

    //Panel Control
    _writeReg(0x90, 0x0010);
    _writeReg(0x92, 0x0000);
    _writeReg(0x93, 0x0000);
    _writeReg(0x95, 0x0110);
    _writeReg(0x97, 0x0000);
    _writeReg(0x98, 0x0000);

    // Display ON
    _writeReg(0x07, D0 | D1 |DTE | GON | BASEE);
} //LCD_ILI9320_Init

static void LCD_SD1289_Init(void)
{
    _writeReg(0x0000,0x0001);
    _writeReg(0x0003,0xA8A4);
    _writeReg(0x000C,0x0000);
    _writeReg(0x000D,0x000C);
    _writeReg(0x000E,0x2B00);
    _writeReg(0x001E,0x00B0);
    _writeReg(0x0001,0x2B3F);
    _writeReg(0x0002,0x0600);
    _writeReg(0x0010,0x0000);
    _writeReg(0x0011,0x6078);
    _writeReg(0x0005,0x0000);
    _writeReg(0x0006,0x0000);
    _writeReg(0x0016,0xEF1C);
    _writeReg(0x0017,0x0003);
    _writeReg(0x0007,0x0133);
    _writeReg(0x000B,0x0000);
    _writeReg(0x000F,0x0000);
    _writeReg(0x0041,0x0000);
    _writeReg(0x0042,0x0000);
    _writeReg(0x0048,0x0000);
    _writeReg(0x0049,0x013F);
    _writeReg(0x004A,0x0000);
    _writeReg(0x004B,0x0000);
    _writeReg(0x0044,0xEF00);
    _writeReg(0x0045,0x0000);
    _writeReg(0x0046,0x013F);
    _writeReg(0x0030,0x0707);
    _writeReg(0x0031,0x0204);
    _writeReg(0x0032,0x0204);
    _writeReg(0x0033,0x0502);
    _writeReg(0x0034,0x0507);
    _writeReg(0x0035,0x0204);
    _writeReg(0x0036,0x0204);
    _writeReg(0x0037,0x0502);
    _writeReg(0x003A,0x0302);
    _writeReg(0x003B,0x0302);
    _writeReg(0x0023,0x0000);
    _writeReg(0x0024,0x0000);
    _writeReg(0x0025,0x8000);
    _writeReg(0x004f,0);
    _writeReg(0x004e,0);
    LCD_DelayMs(10);
}

static void LCD_SPFD5408B_Init(void)
{
    _writeReg(0x0001,0x0100); /* Driver Output Contral Register */
    _writeReg(0x0002,0x0700); /* LCD Driving Waveform Contral */
    _writeReg(0x0003,0x1030); /* Entry Mode */

    _writeReg(0x0004,0x0000); /* Scalling Control register */
    _writeReg(0x0008,0x0207); /* Display Control 2 */
    _writeReg(0x0009,0x0000); /* Display Control 3 */
    _writeReg(0x000A,0x0000); /* Frame Cycle Control */
    _writeReg(0x000C,0x0000); /* External Display Interface Control 1 */
    _writeReg(0x000D,0x0000); /* Frame Maker Position */
    _writeReg(0x000F,0x0000); /* External Display Interface Control 2 */
    LCD_DelayMs(5);
    _writeReg(0x0007,0x0101); /* Display Control */
    LCD_DelayMs(5);
    _writeReg(0x0010,0x16B0); /* Power Control 1 */
    _writeReg(0x0011,0x0001); /* Power Control 2 */
    _writeReg(0x0017,0x0001); /* Power Control 3 */
    _writeReg(0x0012,0x0138); /* Power Control 4 */
    _writeReg(0x0013,0x0800); /* Power Control 5 */
    _writeReg(0x0029,0x0009); /* NVM read data 2 */
    _writeReg(0x002a,0x0009); /* NVM read data 3 */
    _writeReg(0x00a4,0x0000);
    _writeReg(0x0050,0x0000); /* */
    _writeReg(0x0051,0x00EF); /* */
    _writeReg(0x0052,0x0000); /* */
    _writeReg(0x0053,0x013F); /* */

    _writeReg(0x0060,0x2700); /* Driver Output Control */

    _writeReg(0x0061,0x0003); /* Driver Output Control */
    _writeReg(0x006A,0x0000); /* Vertical Scroll Control */

    _writeReg(0x0080,0x0000); /* Display Position Partial Display 1 */
    _writeReg(0x0081,0x0000); /* RAM Address Start Partial Display 1 */
    _writeReg(0x0082,0x0000); /* RAM address End - Partial Display 1 */
    _writeReg(0x0083,0x0000); /* Display Position Partial Display 2 */
    _writeReg(0x0084,0x0000); /* RAM Address Start Partial Display 2 */
    _writeReg(0x0085,0x0000); /* RAM address End Partail Display2 */
    _writeReg(0x0090,0x0013); /* Frame Cycle Control */
    _writeReg(0x0092,0x0000); /* Panel Interface Control 2 */
    _writeReg(0x0093,0x0003); /* Panel Interface control 3 */
    _writeReg(0x0095,0x0110); /* Frame Cycle Control */
    _writeReg(0x0007,0x0173);
}

void LCD_Init(void)
{
    //Power up
    _DispResetOff();
    LCD_DelayMs(100);
    _DispResetOn();
    LCD_DelayMs(10);
    _DispResetOff();
    LCD_DelayMs(10);

    deviceCode = _readReg(0);
    DBGPRINT("LCD device code: 0x%04X\n", deviceCode);
    if (deviceCode == LCD_DEV_9320)
    {
        LCD_ILI9320_Init();
    }
    else if (deviceCode == LCD_DEV_1289)
    {
        LCD_SD1289_Init();
    }
    else if (deviceCode == LCD_DEV_5408)
    {
        LCD_SPFD5408B_Init();
    }
    else
    {
        LCD_ILI9325_Init();
    }

    //Fill display
    LCD_FillAll(LCD_BLACK);

    //Turn on backlight
    _DispBacklightOn();
    DBGPRINT("LCD initialized\n");
}

void LCD_TurnOn(void)
{
    if (deviceCode != LCD_DEV_1289)
    {
        _writeReg(0x10, 0); //Exit standby mode
        _writeReg(0x10, SAP | BT2 | BT1 | APE | AP0); //Enable power supply circuits
    }
    _DispBacklightOn(); //Backlight on
}

void LCD_TurnOff(void)
{
    _DispBacklightOff();  //Backlight off
    if (deviceCode != LCD_DEV_1289)
    {
        _writeReg(0x10, STB); //Standby (Only GRAM continues operation).
    }
}

void LCD_FillRect(LCDPoint p1, LCDPoint p2, LCDColor color)
{
    int32_t numPixels;
    int32_t i;
    uint16_t tmp;

    if (p1.x >= LCD_GetWidth()) p1.x = LCD_GetWidth() - 1;
    if (p2.x >= LCD_GetWidth()) p2.x = LCD_GetWidth() - 1;
    if (p1.y >= LCD_GetHeight()) p1.y = LCD_GetHeight() - 1;
    if (p2.y >= LCD_GetHeight()) p2.y = LCD_GetHeight() - 1;

    if (p1.x > p2.x)
    {
        tmp = p1.x;
        p1.x = p2.x;
        p2.x = tmp;
    }
    if (p1.y > p2.y)
    {
        tmp = p1.y;
        p1.y = p2.y;
        p2.y = tmp;
    }
    numPixels = (p2.x - p1.x + 1) * (p2.y - p1.y + 1);
    _LimitedWindow(p1, p2);
    *BSP_LCD_IDX = 0x22; //Write to GRAM
    for (i = 0; i < numPixels; ++i)
        *BSP_LCD_RAM = color;
    _LimitedWindow(LCD_MakePoint(0,0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1));
}

void LCD_FillAll(LCDColor c)
{
    LCD_FillRect(LCD_MakePoint(0, 0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1), c);
}

LCDColor LCD_MakeRGB(uint8_t r, uint8_t g, uint8_t b)
{
    return LCD_RGB(r, g, b);
}

void LCD_DelayMs(uint32_t ms)
{
    BSP_DelayMs(ms);
}

LCDPoint LCD_MakePoint(int x, int y)
{
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    LCDPoint pt = {x, y};
    return pt;
}

LCDColor LCD_ReadPixel(LCDPoint p)
{
    if (p.x >= LCD_GetWidth() || p.y >= LCD_GetHeight())
        return LCD_BLACK;
    _setCursor(p);
    _writeIdx(0x22); //GRAM
    _readRam(); //Dummy read (invalid data)
    return _readRam();

}

void LCD_InvertPixel(LCDPoint p)
{
    LCD_SetPixel(p, LCD_ReadPixel(p) ^ 0xFFFF);
}

void LCD_SetPixel(LCDPoint p, LCDColor color)
{
    if (p.x >= LCD_GetWidth() || p.y >= LCD_GetHeight())
        return;
    _setCursor(p);
    _writeReg(0x22, color); //Write to GRAM
}

void LCD_Rectangle(LCDPoint a, LCDPoint b, LCDColor c)
{
    LCD_Line(a, LCD_MakePoint(b.x, a.y), c);
    LCD_Line(LCD_MakePoint(b.x, a.y), b, c);
    LCD_Line(b, LCD_MakePoint(a.x, b.y), c);
    LCD_Line(LCD_MakePoint(a.x, b.y), a, c);
}

static void _BresenhamLine(LCDPoint a, LCDPoint b, LCDColor c)
{
    //Bresenham's algorithm
    const short dx = b.x - a.x;
    const short dy = b.y - a.y;
    const short adx = _abs(dx);
    const short ady = _abs(dy);

    assert_param(a.x < LCD_GetWidth());
    assert_param(a.y < LCD_GetHeight());
    assert_param(b.x < LCD_GetWidth());
    assert_param(b.y < LCD_GetHeight());

    _LimitedWindow(LCD_MakePoint(0,0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1));
    if (adx > ady)
    {
        short yincr = dy >= 0 ? 1 : -1;
        short d = 2 * ady - adx;
        short v = 2 * (ady - adx);
        short w = 2 * ady;
        short y = a.y;
        short x;
        if (dx > 0)
        {
            for (x = a.x; x <= b.x; x++)
            {
                LCD_SetPixel(LCD_MakePoint(x, y), c);
                if (d > 0)
                {
                    y += yincr;
                    d += v;
                }
                else
                    d += w;
            }
        }
        else
        {
            for (x = a.x; x >= b.x; x--)
            {
                LCD_SetPixel(LCD_MakePoint(x, y), c);
                if(d > 0)
                {
                    y += yincr;
                    d += v;
                }
                else
                    d += w;
            }
        }
    }
    else
    {
        short xincr = dx >= 0 ? 1 : -1;
        short d = 2 * adx - ady;
        short v = 2 * (adx - ady);
        short w = 2 * adx;
        short x = a.x;
        short y;
        if(dy > 0)
        {
            for (y = a.y; y <= b.y; y++)
            {
                LCD_SetPixel(LCD_MakePoint(x, y), c);
                if(d > 0)
                {
                    x += xincr;
                    d += v;
                }
                else
                    d += w;
            }
        }
        else
        {
            for (y = a.y; y >= b.y; y--)
            {
                LCD_SetPixel(LCD_MakePoint(x, y), c);
                if (d > 0)
                {
                    x += xincr;
                    d += v;
                }
                else
                    d += w;
            }
        }
    }
} // _BresenhamLine

void LCD_Line(LCDPoint a, LCDPoint b, LCDColor color)
{
    int numPixels;
    int i;

    if (a.x >= LCD_GetWidth())
        a.x = LCD_GetWidth() - 1;
    if (b.x >= LCD_GetWidth())
        b.x = LCD_GetWidth() - 1;
    if (a.y >= LCD_GetHeight())
        a.y = LCD_GetHeight() - 1;
    if (b.y >= LCD_GetHeight())
        b.y = LCD_GetHeight() - 1;
    //Horizontal line speed optimization
    if(a.y == b.y)
    {
        _LimitedWindow(LCD_MakePoint(_min(a.x, b.x), a.y),
                       LCD_MakePoint(_max(a.x, b.x), a.y));
        _writeIdx(0x22); //Write to GRAM
        numPixels = _abs(a.x - b.x);
        for (i = 0; i <= numPixels; ++i)
            *BSP_LCD_RAM = color;
        _LimitedWindow(LCD_MakePoint(0,0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1));
        return;
    }

    //Vertical line speed optimization
    if(a.x == b.x)
    {
        _LimitedWindow(LCD_MakePoint(a.x, _min(a.y, b.y)),
                       LCD_MakePoint(a.x, _max(a.y, b.y)));
        _writeIdx(0x22); //Write to GRAM
        numPixels = _abs(a.y - b.y);
        for (i = 0; i <= numPixels; ++i)
            *BSP_LCD_RAM = color;
        _LimitedWindow(LCD_MakePoint(0,0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1));
        return;
    }
    //For non-horizontnal and non-vertical line use slow
    //programmatic way to draw it pixel-by-pixel
    _BresenhamLine(a, b, color);
} //LCD_Line

void LCD_Circle(LCDPoint center, int16_t r, LCDColor color)
{
    int16_t cx = 0;
    int16_t cy = r;
    int16_t df = 1 - r;
    int16_t d_e = 3;
    int16_t d_se = -2 * r + 5;
    if(r == 0)
    {
        return;
    }

    _LimitedWindow(LCD_MakePoint(0,0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1));
    do
    {
        LCD_SetPixel(LCD_MakePoint(center.x + cx, center.y - cy), color);
        LCD_SetPixel(LCD_MakePoint(center.x + cx, center.y + cy), color);
        LCD_SetPixel(LCD_MakePoint(center.x - cx, center.y - cy), color);
        LCD_SetPixel(LCD_MakePoint(center.x - cx, center.y + cy), color);
        LCD_SetPixel(LCD_MakePoint(center.x + cy, center.y + cx), color);
        LCD_SetPixel(LCD_MakePoint(center.x + cy, center.y - cx), color);
        LCD_SetPixel(LCD_MakePoint(center.x - cy, center.y + cx), color);
        LCD_SetPixel(LCD_MakePoint(center.x - cy, center.y - cx), color);

        if (df < 0)
        {
            df += d_e;
            d_e += 2;
            d_se += 2;
        }
        else
        {
            df += d_se;
            d_e += 2;
            d_se += 4;
            cy--;
        }
        cx++;
    }
    while (cx <= cy);
}

void LCD_FillCircle(LCDPoint center, int16_t r, LCDColor color)
{
    int16_t cx = 0;
    int16_t cy = r;
    int16_t df = 1 - r;
    int16_t d_e = 3;
    int16_t d_se = -2 * r + 5;
    if(r == 0)
    {
        return;
    }
    _LimitedWindow(LCD_MakePoint(0,0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1));
    do
    {
        if (df >= 0)
        {
            LCD_Line(LCD_MakePoint(center.x - cx, center.y + cy), LCD_MakePoint(center.x + cx, center.y + cy), color);
            LCD_Line(LCD_MakePoint(center.x - cx, center.y - cy), LCD_MakePoint(center.x + cx, center.y - cy), color);
        }
        if (cx != cy)
        {
            LCD_Line(LCD_MakePoint(center.x - cy, center.y + cx), LCD_MakePoint(center.x + cy, center.y + cx), color);
            LCD_Line(LCD_MakePoint(center.x - cy, center.y - cx), LCD_MakePoint(center.x + cy, center.y - cx), color);
        }
        if (df < 0)
        {
            df += d_e;
            d_e += 2;
            d_se += 2;
        }
        else
        {
            df += d_se;
            d_e += 2;
            d_se += 4;
            cy--;
        }
        cx++;
    }
    while (cx <= cy);
}

//===================================================================
//Drawing bitmap file based on libnsbmp
//===================================================================
#define BYTES_PER_PIXEL 4
#define TRANSPARENT_COLOR 0xFFFFFFFF

void *bitmap_create(int width, int height, unsigned int state)
{
    (void) state;  /* unused */
    return (void*)0x2000F000; //calloc(width * height, BYTES_PER_PIXEL);
}

unsigned char *bitmap_get_buffer(void *bitmap)
{
    return (unsigned char*)bitmap;
}

size_t bitmap_get_bpp(void *bitmap)
{
    (void) bitmap;  /* unused */
    return BYTES_PER_PIXEL;
}

void bitmap_destroy(void *bitmap)
{
}

static LCDPoint _bmpOrigin;

void bitmap_putcolor(unsigned int color32, unsigned int x, unsigned int y)
{
    //DBGPRINT("LCD bmp color: 0x%08x, x: %d, y: %d\n", color32, x, y);
    uint16_t r, g, b;
    r = (uint16_t)(color32 & 0xFF);
    g = (uint16_t)((color32 & 0xFF00) >> 8);
    b = (uint16_t)((color32 & 0xFF0000) >> 16);
    LCDColor clr = LCD_RGB(r, g, b);
    LCD_SetPixel(LCD_MakePoint(_bmpOrigin.x + x, _bmpOrigin.y + y), clr);
}

static bmp_bitmap_callback_vt bitmap_callbacks =
{
    bitmap_create,
    bitmap_destroy,
    bitmap_get_buffer,
    bitmap_get_bpp,
    bitmap_putcolor
};

void LCD_DrawBitmap(LCDPoint origin, const uint8_t *bmpData, uint32_t bmpDataSize)
{
    bmp_image bmp;
    bmp_result code;
    _bmpOrigin = origin;

    bmp_create(&bmp, &bitmap_callbacks);
    code = bmp_analyse(&bmp, bmpDataSize, bmpData);
    if(code != BMP_OK)
    {
        DBGPRINT("bmp_analyse failed, %d\n", code);
        goto cleanup;
    }
    _LimitedWindow(LCD_MakePoint(0,0), LCD_MakePoint(LCD_GetWidth() - 1, LCD_GetHeight() - 1));

    code = bmp_decode(&bmp);
    if(code != BMP_OK)
    {
        DBGPRINT("bmp_decode failed, %d\n", code);
        if(code != BMP_INSUFFICIENT_DATA)
        {
            goto cleanup;
        }
    }
cleanup:
    bmp_finalise(&bmp);
}

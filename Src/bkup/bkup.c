/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <stdint.h>

#include "stm32f10x.h"

#define BKP_DR_NUMBER              42

//Registers assignment
#define BK_FMEAS_LO 2
#define BK_FMEAS_HI 3
#define BK_FGEN_LO 4
#define BK_FGEN_HI 5
#define BK_F1_LO 6
#define BK_F1_HI 7
#define BK_SPAN  8
#define BK_XTAL_CORR 9

static const uint16_t BKUP_Reg[BKP_DR_NUMBER] =
{
    BKP_DR1, BKP_DR2, BKP_DR3, BKP_DR4, BKP_DR5, BKP_DR6, BKP_DR7, BKP_DR8,
    BKP_DR9, BKP_DR10, BKP_DR11, BKP_DR12, BKP_DR13, BKP_DR14, BKP_DR15, BKP_DR16,
    BKP_DR17, BKP_DR18, BKP_DR19, BKP_DR20, BKP_DR21, BKP_DR22, BKP_DR23, BKP_DR24,
    BKP_DR25, BKP_DR26, BKP_DR27, BKP_DR28, BKP_DR29, BKP_DR30, BKP_DR31, BKP_DR32,
    BKP_DR33, BKP_DR34, BKP_DR35, BKP_DR36, BKP_DR37, BKP_DR38, BKP_DR39, BKP_DR40,
    BKP_DR41, BKP_DR42
};

void BKUP_Initialize(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
}

void BKUP_Write16(int reg, uint16_t data)
{
    PWR_BackupAccessCmd(ENABLE);
    BKP_WriteBackupRegister(BKUP_Reg[reg], data);
}

uint16_t BKUP_Read16(int reg)
{
    return BKP_ReadBackupRegister(BKUP_Reg[reg]);
}

uint32_t BKUP_LoadPanF1(void)
{
    uint32_t fbkup = BKUP_Read16(BK_F1_LO) & 0xFFFF;
    fbkup |= (((uint32_t)BKUP_Read16(BK_F1_HI)) << 16);
    return fbkup;
}

uint16_t BKUP_LoadSpan(void)
{
    return BKUP_Read16(BK_SPAN);
}

//Save F1 and span for panoramic window
void BKUP_SaveF1Span(uint32_t f1, int span)
{
    BKUP_Write16(BK_F1_LO, f1 & 0xFFFF);
    BKUP_Write16(BK_F1_HI, f1 >> 16);
    BKUP_Write16(BK_SPAN, (uint16_t)span);
}

uint32_t BKUP_LoadFMeas(void)
{
    uint32_t fbkup = BKUP_Read16(BK_FMEAS_LO) & 0xFFFF;
    fbkup |= (((uint32_t)BKUP_Read16(BK_FMEAS_HI)) << 16);
    return fbkup;
}

void BKUP_SaveFMeas(uint32_t fMeas)
{
    BKUP_Write16(BK_FMEAS_LO, fMeas & 0xFFFF);
    BKUP_Write16(BK_FMEAS_HI, fMeas >> 16);
}

uint32_t BKUP_LoadFGen(void)
{
    uint32_t fbkup = BKUP_Read16(BK_FGEN_LO) & 0xFFFF;
    fbkup |= (((uint32_t)BKUP_Read16(BK_FGEN_HI)) << 16);
    return fbkup;
}

void BKUP_SaveFGen(uint32_t fGen)
{
    BKUP_Write16(BK_FGEN_LO, fGen & 0xFFFF);
    BKUP_Write16(BK_FGEN_HI, fGen >> 16);
}

int BKUP_LoadXtalCorr(void)
{
    int res = BKUP_Read16(BK_XTAL_CORR);
    if (res == 0xFFFF)
        res = 0;
    else
        res -= 0x7FFF;
    if (res < -7000)
        res = -7000;
    else if (res > 7000)
        res = 7000;
    return res;
}

void BKUP_SaveXtalCorr(int corr)
{
    corr += 0x7FFF;
    uint16_t wValue  = (uint16_t)(corr & 0xFFFF);
    BKUP_Write16(BK_XTAL_CORR, wValue);
}

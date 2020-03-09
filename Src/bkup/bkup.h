/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef BKUP_H_INCLUDED
#define BKUP_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BKUP_Initialize(void);
void BKUP_Write16(int reg, uint16_t data);
uint16_t BKUP_Read16(int reg);

uint32_t BKUP_LoadFMeas(void);
void BKUP_SaveFMeas(uint32_t fMeas);
uint32_t BKUP_LoadPanF1(void);
uint16_t BKUP_LoadSpan(void);
void BKUP_SaveF1Span(uint32_t f1, int span);
uint32_t BKUP_LoadFGen(void);
void BKUP_SaveFGen(uint32_t fGen);
void BKUP_SaveXtalCorr(int corr);
int BKUP_LoadXtalCorr(void);

#ifdef __cplusplus
}
#endif

#endif //BKUP_H_INCLUDED

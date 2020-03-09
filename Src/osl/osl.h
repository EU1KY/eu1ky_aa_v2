/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef _OSL_H_
#define _OSL_H_

#include <stdint.h>
#include <complex.h>

typedef float complex COMPLEX;

#pragma pack(push, 1)
//Hardware error correction sctucture
typedef struct
{
    float mag0;    //Magnitude ratio correction coefficient
    float phase0;  //Phase correction value
} OSL_ERRCORR;
#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

extern float OSL_RSHORT;
extern const float OSL_RLOAD;
extern float OSL_ROPEN;

void OSL_Select(int index);

int OSL_CleanOSL(void);
int OSL_GetSelected(void);

int OSL_ScanShort(void);
int OSL_ScanLoad(void);
int OSL_ScanOpen(void);
int OSL_Calculate(void);
void OSL_RecalcLoads(void);

int OSL_ScanADCCorrection(void);

OSL_ERRCORR OSL_GetADCCorrection(uint32_t fhz);
COMPLEX OSL_CorrectG(uint32_t fhz, COMPLEX gMeasured);
COMPLEX OSL_CorrectZ(uint32_t fhz, COMPLEX zMeasured);

COMPLEX OSL_GFromZ(COMPLEX Z);
COMPLEX OSL_ZFromG(COMPLEX G);

#ifdef __cplusplus
}
#endif

#endif

/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef _TOUCH_H_
#define _TOUCH_H_

#include "LCD.h"

struct TSCAL_PARAMS
{
    uint32_t xmin;
    uint32_t xmax;
    uint32_t ymin;
    uint32_t ymax;
    uint32_t reversed_xy;
    uint32_t invert_x;
    uint32_t invert_y;
    uint32_t xbw;
    uint32_t ybw;
};

extern const struct TSCAL_PARAMS *TOUCH_cal;

#ifdef __cplusplus
extern "C" {
#endif

void TOUCH_Init(void);

int TOUCH_Poll(LCDPoint* pCoord);

int TOUCH_IsPressed(void);

unsigned int TOUCH_GetRawX(void);

unsigned int TOUCH_GetRawY(void);

#ifdef __cplusplus
}
#endif
#endif //_TOUCH_H_


/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#ifndef BTUSART_H_INCLUDED
#define BTUSART_H_INCLUDED

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BTUSART_Init(void);
int BTUSART_Putchar(int ch);
int BTUSART_Getchar(void);
int BTUSART_PutString(const char* str);
int BTUSART_PutBytes(const unsigned char* bytes, int len);
uint32_t BTUSART_GetRxOvfCount(void);
uint32_t BTUSART_GetTxOvfCount(void);
int BTUSART_IsTxBusy(void);

#ifdef __cplusplus
}
#endif

#endif //BTUSART_H_INCLUDED

/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "config.h"
#include "stm32f10x.h"
#include "btusart.h"
#include "fifo.h"

static FIFO_Descr rxfifo, txfifo;
static volatile int btusart_busy = 0;
static volatile uint32_t rx_overflow_ctr = 0;
static volatile uint32_t tx_overflow_ctr = 0;

void BTUSART_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    FIFO_Init(&rxfifo);
    FIFO_Init(&txfifo);

    //if (IS_RCC_APB2_PERIPH(BTUSART_RCC))
        RCC_APB2PeriphClockCmd(BTUSART_RCC, ENABLE);
    //else
        RCC_APB1PeriphClockCmd(BTUSART_RCC, ENABLE);

    RCC_APB2PeriphClockCmd(BTUSART_GPIO_APB2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);

    // Configure USART Rx as input floating
    GPIO_InitStructure.GPIO_Pin = BTUSART_PIN_Rx;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BTUSART_GPIO, &GPIO_InitStructure);

    // Configure USART Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = BTUSART_PIN_Tx;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BTUSART_GPIO, &GPIO_InitStructure);

    //Configure NVIC
    NVIC_InitStructure.NVIC_IRQChannel = BTUSART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_Init(&NVIC_InitStructure);

    // Configure USART
    USART_InitStructure.USART_BaudRate = BTUSART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(BTUSART_USED, &USART_InitStructure);

    // Enable the USART
    USART_Cmd(BTUSART_USED, ENABLE);

    // Enable USART Receive and Transmit interrupts
    USART_ITConfig(BTUSART_USED, USART_IT_RXNE, ENABLE);
    USART_ITConfig(BTUSART_USED, USART_IT_TC, ENABLE);
}

//======================================================
void BTUSART_IRQHandler(void)
{
    uint8_t byte;

    //Tx complete interrupt
    if(USART_GetITStatus(BTUSART_USED, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(BTUSART_USED, USART_IT_TC);
        if (FIFO_OK != FIFO_Get(&txfifo, &byte))
        {//All bytes have been transmitted
            btusart_busy = 0;
        }
        else
        {
            BTUSART_USED->DR = byte;
        }
    }

    //Receive Data register not empty interrupt
    if(USART_GetITStatus(BTUSART_USED, USART_IT_RXNE) != RESET)
    {
        // Read one byte from the receive data register
        byte = (uint8_t)BTUSART_USED->DR;
        /*
        if (byte == '\r')
            printf("\n");
        else
            printf("%c", byte);
        */
        if (FIFO_OK != FIFO_Put(&rxfifo, byte))
        {
            rx_overflow_ctr++;
        }
    }
}

//======================================================
extern void Sleep(uint32_t nms);

int BTUSART_Putchar (int ch)
{
    FIFO_STATUS res;

    __disable_irq();
    res = FIFO_Put(&txfifo, (uint8_t)ch);
    __enable_irq();
    if (res)
    {
        while(btusart_busy && txfifo.count == FIFO_SIZE); //Wait until space in the buffer appears
                                          //Warning: it is a potential place to hang, but this
                                          //should not happen if STM32 USART hardware works OK.
        __disable_irq();
        res = FIFO_Put(&txfifo, (uint8_t)ch);
        __enable_irq();
        if (res)
        {
            tx_overflow_ctr++;
            return res;
        }
    }

    //start transmission if there's something in the buffer and transmission is off
    if (!btusart_busy && !FIFO_IsEmpty(&txfifo))
    {
        uint8_t ch;
        __disable_irq();
        FIFO_Get(&txfifo, &ch);
        __enable_irq();
        btusart_busy = 1;
        BTUSART_USED->DR = ch; //Start transmission
    }
    return 0;
}

//======================================================
int BTUSART_Getchar(void)
{
    uint8_t ch;
    if (FIFO_Get(&rxfifo, &ch))
        return EOF;
    return (int)ch;
}

//======================================================
int BTUSART_PutString(const char* str)
{
    return BTUSART_PutBytes((const unsigned char*)str, strlen(str));
}

//======================================================
int BTUSART_PutBytes(const unsigned char* bytes, int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        BTUSART_Putchar(bytes[i]);
    }
    return len;
}

//======================================================
uint32_t BTUSART_GetRxOvfCount(void)
{
    return rx_overflow_ctr;
}

//======================================================
uint32_t BTUSART_GetTxOvfCount(void)
{
    return tx_overflow_ctr;
}

//============================================
int BTUSART_IsTxBusy(void)
{
    return btusart_busy;
}

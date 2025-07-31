#ifndef __USART_H__
#define __USART_H__

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOSConfig.h"
#include <stdarg.h>
#include <string.h>
#include "global.h"
#include "data_protocol.h"

extern uint8_t receive_Buff1[RX_BUFFER_SIZE]; //接收数据直接区
extern uint8_t handle_Buff1[RX_BUFFER_SIZE]; //接收数据缓冲区
 
void uart_init(void); //串口初始化函数
void USAR_UART_IDLECallback(void); //串口中断回调函数
void debug_printf(const char *fmt, ...);//打印函数
void UART_Transmit_DMA_Safe(uint8_t *data, uint16_t len);// 线程安全的DMA发送函数调函数


#endif


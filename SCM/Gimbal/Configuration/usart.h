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

extern uint8_t receive_Buff1[RX_BUFFER_SIZE]; //��������ֱ����
extern uint8_t handle_Buff1[RX_BUFFER_SIZE]; //�������ݻ�����
 
void uart_init(void); //���ڳ�ʼ������
void USAR_UART_IDLECallback(void); //�����жϻص�����
void debug_printf(const char *fmt, ...);//��ӡ����
void UART_Transmit_DMA_Safe(uint8_t *data, uint16_t len);// �̰߳�ȫ��DMA���ͺ���������


#endif


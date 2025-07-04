/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stm32f1xx_hal_dma.h"
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
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


#ifndef _ELETUBE_H_

#define _ELETUBE_H_

  

#include "stm32f1xx.h"

#include <stdio.h>

  
extern uint8_t etubeCkeck[5];

//光电对管中断输入引脚

#define ETUBE_PIN_1  GPIO_PIN_0

#define ETUBE_PIN_2  GPIO_PIN_2

#define ETUBE_PIN_3  GPIO_PIN_3

#define ETUBE_PIN_4  GPIO_PIN_4

#define ETUBE_PIN_5  GPIO_PIN_5

  

#define ETUBE_PORT_1 GPIOB

#define ETUBE_PORT_2 GPIOA

#define ETUBE_PORT_3 GPIOA

#define ETUBE_PORT_4 GPIOA

#define ETUBE_PORT_5 GPIOA

  

void Etube_Check(void);//将探测情况输出

  

#endif


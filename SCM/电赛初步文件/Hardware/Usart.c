#include "stm32f10x.h"                  // Device header

uint8_t Rx_Data;
uint8_t Rx_Flag;
void Usart1_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	//tx
	GPIO_InitTypeDef	GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	USART_InitTypeDef	USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_InitTypeDef	NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
}

void Usart1_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
}

void Usart1_SendArray(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i++)
	{
		Usart1_SendByte(Array[i]);
	}
}

void Usart1_SendString(char *String)
{
	uint8_t i;
	for(i = 0; String[i] != '\0';i++)
	{
		Usart1_SendByte(String[i]);
	}
}

//��ֵת��
uint32_t Usart1_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

void Usart1_SendNum(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		Usart1_SendByte(Number / Usart1_Pow(10, Length - i - 1) % 10 + '0');
	}
}

uint8_t Usart1_GetRxFlag()
{
	if (Rx_Flag == 1)
	{
		Rx_Flag = 0;
		return 1;
	}
	return 0;
}

uint8_t Usart1_GetRxData()
{
	return Rx_Data;
}

void USART1_IRQHandler()
{
	if (USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET)
	{
		Rx_Data = USART_ReceiveData(USART1);
		Rx_Flag = 1;
		USART_ClearITPendingBit(USART1,USART_FLAG_RXNE);
	}
}

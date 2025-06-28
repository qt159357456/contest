#ifndef _USART_H_
#define _USART_H_

void Usart1_Init(void);
void Usart1_SendByte(uint8_t Byte);
uint8_t Usart1_GetRxFlag(void);
uint8_t Usart1_GetRxData(void);
void Usart1_SendArray(uint8_t *Array,uint16_t Length);
void Usart1_SendString(char *String);
void Usart1_SendNum(uint32_t Number, uint8_t Length);
#endif

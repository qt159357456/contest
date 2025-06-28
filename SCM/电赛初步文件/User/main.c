#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "PWM.h"
#include "Key.h"
#include "IC.h"
#include "OLED.h"
#include "Usart.h"

uint8_t Tx_Array[4];
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	Usart1_Init();
	OLED_Init();
	OLED_ShowString(1,1,"Rx_Data:");
	Tx_Array[0] = 10;
	Tx_Array[1] = 45;
	Tx_Array[2] = 23;
	Tx_Array[3] = 65;
//	
//	Usart1_SendArray(Tx_Array,4);
//	Usart1_SendString("YourFun!");
//	Usart1_SendNum(12345,5);
	while(1)
	{
		if (Usart1_GetRxFlag())
		{
			OLED_ShowHexNum(1,9,Usart1_GetRxData(),2);
		}
		Tx_Array[0]++;
		Tx_Array[1]++;
		Tx_Array[2]++;
		Tx_Array[3]++;	
		
		Usart1_SendArray(Tx_Array,4);
		Delay_ms(1);
	}
}



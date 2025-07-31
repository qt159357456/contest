#include "keys.h"

uint8_t point = 1;
int redir;
int keyflag = 0;
int keyflag2 = 0;
void key1(void){
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_SET){
					point=1;
					openmv_send_command(&point,1);
        }
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_SET)
						vTaskDelay(1);
			}
}

void key2(void){
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET){
					Move_Absolute_Angle_Y(0x01,50,30);
					Move_Absolute_Angle_X(0x01,20,20);
        }
					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

void key3(void){
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET){
						Motor_Enable_Y(0x01,0x00,0);
        }
					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

void key4(void){
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_SET){
						
        }
					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

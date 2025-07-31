#include "keys.h"

uint8_t point = 2;
int redir;
int keyflag = 0;
int keyflag2 = 0;
void key1(void){
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_SET){
							if(key < 5){
									openmv_send_command(&point,1);
									key++;
							}
              else{
									keyflag2 = 1;
									Reset_Position();
							}
							if(key==5){
									Step_Init();
							}
        }
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_SET)
					if(keyflag2)
						keyflag=1;
					vTaskDelay(1);
			}
}

void key2(void){
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET){
						redir ^= 1;
        }
					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

void key3(void){
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET){
						move_one_step(&motor_x,redir,1000);
        }
					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

void key4(void){
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_SET){
					move_one_step(&motor_y,redir,1000);	
        }
					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

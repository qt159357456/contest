#include "keys.h"

uint8_t point = 1;
int redir;
int keyflag = 0;
void key1(void){
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_SET){
							if(key < 5){
									openmv_send_command(&point,1);
									key++;
							}
              else{
									Reset_Position();
							}
							if(key==5){
									Step_Init();
							}
        }
					while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==GPIO_PIN_SET)
					vTaskDelay(1);
			}
}

void key2(void){
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET){
						redir ^= 1;
						keyflag = 1;
        }
					while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

void key3(void){
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==GPIO_PIN_SET){
						move_one_step(&motor_x,redir,1000);
						keyflag = 1;
        }
					while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

void key4(void){
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==GPIO_PIN_SET){
        vTaskDelay(100);
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==GPIO_PIN_SET){
					move_one_step(&motor_y,redir,1000);	
					keyflag = 1;
        }
					while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==GPIO_PIN_SET)
					vTaskDelay(1);
			}	
}

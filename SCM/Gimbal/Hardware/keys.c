#include "keys.h"

uint8_t point = 1;
int keyflag = 0;
//void key1(void){
//			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_RESET){
//        vTaskDelay(100);
//        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_RESET){
//					point=1;
//					openmv_send_command(&point,1);
//        }
//				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)==GPIO_PIN_RESET)
//						vTaskDelay(1);
//			}
//}

//void key2(void){
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_RESET){
//        vTaskDelay(100);
//        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_RESET){
//					point=2;
//					openmv_send_command(&point,1);
//        }
//					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_RESET)
//					vTaskDelay(1);
//			}	
//}

//void key3(void){
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_RESET){
//        vTaskDelay(100);
//        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_RESET){
//					point=3;
//					openmv_send_command(&point,1);
//        }
//					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)==GPIO_PIN_RESET)
//					vTaskDelay(1);
//			}	
//}

//void key4(void){
//			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_RESET){
//        vTaskDelay(100);
//        if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_RESET){
//						
//        }
//					while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==GPIO_PIN_RESET)
//					vTaskDelay(1);
//			}	
//}

int task_num = 0;
void key1(void){
			if(task_num==1){
				point=1;
				openmv_send_command(&point,1);
				task_num = 0;
			}
}

void key2(void){
			if(task_num==2){
				point=2;
				openmv_send_command(&point,1);
				task_num = 0;
			}
}

void key3(void){
			if(task_num==3){
				point=3;
				openmv_send_command(&point,1);
				task_num = 0;
			}
}

void key4(void){
			if(task_num==4){
				point=4;
				openmv_send_command(&point,1);
				task_num = 0;
			}
}

void key5(void){
			if(task_num==5){
				point=5;
				openmv_send_command(&point,1);
				task_num = 0;
			}
}


void enable_laser(uint8_t enable){
			if(enable){
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
			}
}

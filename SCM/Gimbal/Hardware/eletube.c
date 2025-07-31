#include "eletube.h"

//  

//uint8_t etubeCkeck[5] = {1,1,1,1,1};//存放每个对管探测到黑线的状态，1没探测到，0探测到

//  

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//输入中断处理函数

//{

//    if(GPIO_Pin == ETUBE_PIN_1)//1号引脚

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_1, ETUBE_PIN_1))//上升沿

//        {

//            etubeCkeck[0] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_1, ETUBE_PIN_1))//下降沿

//        {

//            etubeCkeck[0] = 0;

//        }

//    }

//  

//    else if(GPIO_Pin == ETUBE_PIN_2)//2号引脚

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_2, ETUBE_PIN_2))//上升沿

//        {

//            etubeCkeck[1] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_2, ETUBE_PIN_2))//下降沿

//        {

//            etubeCkeck[1] = 0;

//        }

//    }

//  

//    else if(GPIO_Pin == ETUBE_PIN_3)//3号引脚

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_3, ETUBE_PIN_3))//上升沿

//        {

//            etubeCkeck[2] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_3, ETUBE_PIN_3))//下降沿

//        {

//            etubeCkeck[2] = 0;

//        }

//    }

//    else if(GPIO_Pin == ETUBE_PIN_4)//4号引脚

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_4, ETUBE_PIN_4))//上升沿

//        {

//            etubeCkeck[3] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_4, ETUBE_PIN_4))//下降沿

//        {

//            etubeCkeck[3] = 0;

//        }

//    }

//    else if(GPIO_Pin == ETUBE_PIN_5)//5号引脚

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_5, ETUBE_PIN_5))//上升沿

//        {

//            etubeCkeck[4] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_5, ETUBE_PIN_5))//下降沿

//        {

//            etubeCkeck[4] = 0;

//        }

//    }

//}

//  

//void Etube_Check(void)//将探测情况输出

//{
//    uint8_t i = 0;

//    for(i = 0;i < 5;i++)

//    {

//        if(etubeCkeck[i] == 0) printf("0");

//        else if(etubeCkeck[i] == 1) printf("1");
//				if(i!=4)
//						printf(",");
//        

//    }
//		printf("\n");

//}


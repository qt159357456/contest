#include "eletube.h"

//  

//uint8_t etubeCkeck[5] = {1,1,1,1,1};//���ÿ���Թ�̽�⵽���ߵ�״̬��1û̽�⵽��0̽�⵽

//  

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//�����жϴ�����

//{

//    if(GPIO_Pin == ETUBE_PIN_1)//1������

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_1, ETUBE_PIN_1))//������

//        {

//            etubeCkeck[0] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_1, ETUBE_PIN_1))//�½���

//        {

//            etubeCkeck[0] = 0;

//        }

//    }

//  

//    else if(GPIO_Pin == ETUBE_PIN_2)//2������

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_2, ETUBE_PIN_2))//������

//        {

//            etubeCkeck[1] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_2, ETUBE_PIN_2))//�½���

//        {

//            etubeCkeck[1] = 0;

//        }

//    }

//  

//    else if(GPIO_Pin == ETUBE_PIN_3)//3������

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_3, ETUBE_PIN_3))//������

//        {

//            etubeCkeck[2] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_3, ETUBE_PIN_3))//�½���

//        {

//            etubeCkeck[2] = 0;

//        }

//    }

//    else if(GPIO_Pin == ETUBE_PIN_4)//4������

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_4, ETUBE_PIN_4))//������

//        {

//            etubeCkeck[3] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_4, ETUBE_PIN_4))//�½���

//        {

//            etubeCkeck[3] = 0;

//        }

//    }

//    else if(GPIO_Pin == ETUBE_PIN_5)//5������

//    {

//        if(HAL_GPIO_ReadPin(ETUBE_PORT_5, ETUBE_PIN_5))//������

//        {

//            etubeCkeck[4] = 1;

//        }

//        if(!HAL_GPIO_ReadPin(ETUBE_PORT_5, ETUBE_PIN_5))//�½���

//        {

//            etubeCkeck[4] = 0;

//        }

//    }

//}

//  

//void Etube_Check(void)//��̽��������

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


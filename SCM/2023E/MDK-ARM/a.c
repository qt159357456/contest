#include "usart.h"
 
// ��������ֱ����
uint8_t receive_Buff1[BUFF_NUM];
// �������ݻ�����
uint8_t handle_Buff1[BUFF_NUM];
// ���ڼ����ַ�����
extern DMA_HandleTypeDef hdma_usart1_rx;
 
// ���ڳ�ʼ��
void uart_init(void)
{
    // ��������1�Ĵ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // ����1������Ϣ��ʼ������ʱ�յ��Ľ����洢��receive_Buff1��
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_Buff1, BUFF_NUM);
}
 
/*
 * �жϴ���ص�����
 */
void USAR_UART_IDLECallback(void)
{
    // �ж��Ƿ��ǿ����ж�
    if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        // ��������жϱ�־�������һֱ���Ͻ����жϣ�
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // ��ʱ�ر�DMA
        HAL_UART_DMAStop(&huart1);
        // ������յ������ݳ���
        uint8_t receive_len1  = BUFF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        // �ѽ��ջ����е����ݸ��Ƶ���������
        memcpy(handle_Buff1, receive_Buff1, BUFF_NUM);
        // printf("%s\r\n", handle_Buff1); // ���Ժ����������յ������ݴ�ӡ��ȥ
        Data_Handle1();
        // ��ս��ջ�����
        memset(receive_Buff1, 0, receive_len1);
        // ���¿���DMA
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_Buff1, BUFF_NUM);
    }
}

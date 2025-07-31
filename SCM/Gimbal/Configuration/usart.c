#include "usart.h"

// ��������ֱ����
uint8_t receive_Buff1[RX_BUFFER_SIZE];
// �������ݻ�����
uint8_t handle_Buff1[RX_BUFFER_SIZE];
// ���ڼ����ַ�����
extern DMA_HandleTypeDef hdma_usart1_rx;


uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_busy = 0;
 
// ���ڳ�ʼ��
void uart_init(void)
{
//    // ��������1�Ĵ����ж�
//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//    // ����1������Ϣ��ʼ������ʱ�յ��Ľ����洢��receive_Buff1��
//        HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_Buff1, RX_BUFFER_SIZE);
	
	// ��������1�Ĵ����ж�
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // ����DMA��������ж�
    __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);
    // ����1������Ϣ��ʼ��
    HAL_UART_Receive_DMA(&huart1, receive_Buff1, RX_BUFFER_SIZE);
}


void debug_printf(const char *fmt, ...) {
    char buf[TX_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    
    // �ڴ˴����öϵ�
	UART_Transmit_DMA_Safe((uint8_t*)buf,strlen(buf));
//    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

// �̰߳�ȫ��DMA���ͺ���
void UART_Transmit_DMA_Safe(uint8_t *data, uint16_t len)
{
    
    if(len > TX_BUFFER_SIZE) len = TX_BUFFER_SIZE;
    
    // �������ݵ����ͻ�����
    memcpy(tx_buffer, data, len);
    
    // ����DMA����
    HAL_UART_Transmit_DMA(&huart1, tx_buffer, len);
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
        uint8_t receive_len1  = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        // �ѽ��ջ����е����ݸ��Ƶ���������
        memcpy(handle_Buff1, receive_Buff1, RX_BUFFER_SIZE);
        // printf("%s\r\n", handle_Buff1); // ���Ժ����������յ������ݴ�ӡ��ȥ
        Data_Handle1();
        // ��ս��ջ�����
        memset(receive_Buff1, 0, receive_len1);
        // ���¿���DMA
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_Buff1, RX_BUFFER_SIZE);
    }
}

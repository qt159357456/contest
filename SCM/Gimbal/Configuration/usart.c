#include "usart.h"

// ��������ֱ����
uint8_t receive_Buff1[RX_BUFFER_SIZE];
// �������ݻ�����
uint8_t handle_Buff1[RX_BUFFER_SIZE];
// ���ڼ����ַ�����
extern DMA_HandleTypeDef hdma_usart1_rx;



// ��������ֱ����
uint8_t receive_Buff2[RX_BUFFER_SIZE];
// �������ݻ�����
uint8_t handle_Buff2[RX_BUFFER_SIZE];
extern DMA_HandleTypeDef hdma_usart2_rx;


// ��������ֱ����
uint8_t receive_Buff4[RX_BUFFER_SIZE];
// �������ݻ�����
uint8_t handle_Buff4[RX_BUFFER_SIZE];
extern DMA_HandleTypeDef hdma_uart4_rx;


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
	
	
	// ��������2�Ĵ����ж�
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    // ����DMA��������ж�
    __HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
    // ����2������Ϣ��ʼ��
    HAL_UART_Receive_DMA(&huart2, receive_Buff2, RX_BUFFER_SIZE);
	
	
	// ��������4�Ĵ����ж�
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    // ����DMA��������ж�
    __HAL_DMA_ENABLE_IT(&hdma_uart4_tx, DMA_IT_TC);
    // ����4������Ϣ��ʼ��
    HAL_UART_Receive_DMA(&huart4, receive_Buff4, RX_BUFFER_SIZE);
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

/* ���͵������� */
volatile uint8_t uart_tx_busy = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
		if (huart->Instance == USART2) {  // ����ʵ��UART�޸�
        uart_tx_busy = 0;  // ���æµ��־
    }
}
HAL_StatusTypeDef Send_Motor_Command_Y(uint8_t* data, uint16_t size) {
    return HAL_UART_Transmit_DMA(&Y_MOTOR_HUART, data, size);
}

HAL_StatusTypeDef Send_Motor_Command_X(uint8_t* data, uint16_t size) {
    return HAL_UART_Transmit_DMA(&X_MOTOR_HUART, data, size);
}
/*
 * �жϴ���ص�����
 */
void USAR_UART_IDLECallback(UART_HandleTypeDef* huart)
{
    // �ж��Ƿ��ǿ����ж�
    if (huart==&huart1&&RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
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
		else if (huart==&huart2&&RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)){
        // ��������жϱ�־�������һֱ���Ͻ����жϣ�
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        // ��ʱ�ر�DMA
        HAL_UART_DMAStop(&huart2);
        // ������յ������ݳ���
        uint8_t receive_len2  = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
        // �ѽ��ջ����е����ݸ��Ƶ���������
        memcpy(handle_Buff2, receive_Buff2, RX_BUFFER_SIZE);
        // printf("%s\r\n", handle_Buff1); // ���Ժ����������յ������ݴ�ӡ��ȥ
        Data_Handle2();
        // ��ս��ջ�����
        memset(receive_Buff2, 0, receive_len2);
        // ���¿���DMA
        HAL_UART_Receive_DMA(&huart2, (uint8_t*)receive_Buff2, RX_BUFFER_SIZE);
    }
		
		else if (huart==&huart4&&RESET != __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE)){
        // ��������жϱ�־�������һֱ���Ͻ����жϣ�
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        // ��ʱ�ر�DMA
        HAL_UART_DMAStop(&huart4);
        // ������յ������ݳ���
        uint8_t receive_len4  = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
        // �ѽ��ջ����е����ݸ��Ƶ���������
        memcpy(handle_Buff4, receive_Buff4, RX_BUFFER_SIZE);
        // printf("%s\r\n", handle_Buff1); // ���Ժ����������յ������ݴ�ӡ��ȥ
        Data_Handle4();
        // ��ս��ջ�����
        memset(receive_Buff4, 0, receive_len4);
        // ���¿���DMA
        HAL_UART_Receive_DMA(&huart4, (uint8_t*)receive_Buff2, RX_BUFFER_SIZE);
    }
}

#include "usart.h"
 
// 接收数据直接区
uint8_t receive_Buff1[BUFF_NUM];
// 接收数据缓冲区
uint8_t handle_Buff1[BUFF_NUM];
// 用于计算字符长度
extern DMA_HandleTypeDef hdma_usart1_rx;
 
// 串口初始化
void uart_init(void)
{
    // 开启串口1的串口中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    // 串口1接受消息初始化，此时收到的结果会存储到receive_Buff1中
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_Buff1, BUFF_NUM);
}
 
/*
 * 中断处理回调函数
 */
void USAR_UART_IDLECallback(void)
{
    // 判断是否是空闲中断
    if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        // 清除空闲中断标志（否则会一直不断进入中断）
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // 暂时关闭DMA
        HAL_UART_DMAStop(&huart1);
        // 计算接收到的数据长度
        uint8_t receive_len1  = BUFF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        // 把接收缓冲中的数据复制到处理缓冲中
        memcpy(handle_Buff1, receive_Buff1, BUFF_NUM);
        // printf("%s\r\n", handle_Buff1); // 测试函数：将接收到的数据打印出去
        Data_Handle1();
        // 清空接收缓冲区
        memset(receive_Buff1, 0, receive_len1);
        // 重新开启DMA
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_Buff1, BUFF_NUM);
    }
}

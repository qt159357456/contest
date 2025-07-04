#ifndef __DATA_PROTOCOL_H
#define __DATA_PROTOCOL_H
#include "global.h"
#include "mutex_lock_and_message_queue.h"
#include "usart.h"

static void parse_openmv_frame(OpenMVFrame_t* frame);
void openmv_send_command(uint8_t cmd, const uint8_t* data, uint8_t len);
void Data_Handle1(void);


extern uint8_t openmv_rx_buf[OPENMV_RX_BUFFER_SIZE];
extern uint16_t openmv_rx_len;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
// OpenMVÊý¾Ý¼°»¥³âËø
extern OpenMVData_t g_openmv_data;
extern CustomMutex_t openmv_data_mutex;
#endif


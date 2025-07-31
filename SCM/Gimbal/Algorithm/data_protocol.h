#ifndef __DATA_PROTOCOL_H
#define __DATA_PROTOCOL_H
#include "global.h"
#include "mutex_lock_and_message_queue.h"
#include "usart.h"
#include "myMath.h"

static void parse_openmv_frame(OpenMVFrame_RX_t* frame);
void openmv_send_command(const uint8_t* data, uint8_t len);
void Data_Handle1(void);
void Data_Handle2(void);
void Data_Handle4(void); 

extern uint8_t openmv_rx_buf[OPENMV_RX_BUFFER_SIZE];
extern uint16_t openmv_rx_len;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
// OpenMVÊý¾Ý¼°»¥³âËø
extern OpenMVData_t g_openmv_data;
extern CustomMutex_t openmv_data_mutex;

extern int key;
extern float angle_pitch[5],angle_yaw[5];
extern float target_x[5],target_y[5];
extern uint16_t xtar,ytar,xcur,ycur;
extern int16_t offset_x,offset_y;
extern uint8_t send_speed_x_flag,send_speed_y_flag;
#endif


#include "data_protocol.h"
#include "motor.h"

// 串口接收缓冲区
uint8_t openmv_rx_buf[128] = {0};
uint16_t openmv_rx_len = 0;
// OpenMV数据及互斥锁
//OpenMVData_t g_openmv_data = {TARGET_NONE};
CustomMutex_t openmv_data_mutex = {
    .locked = 0,
    .owner = NULL,
    .name = "openmv_data_mutex",
    .lock_count = 0
};

uint8_t my_flag;
float target_x[5] = {-235,250,250,-235,0},target_y[5] = {230,230,-250,-250,0};
float angle_pitch[5],angle_yaw[5];
int key;
uint8_t cmd = 2;

int16_t offset_x,offset_y;
int flagx,flagy;
float kpx,kpy;
// 解析完整的OpenMV帧，采集，决策，驱动
static void parse_openmv_frame(OpenMVFrame_RX_t* frame) {
//		my_flag = frame->data[0];
//				targetx = frame->data[1]<<8|frame->data[2];
//				targety = frame->data[3]<<8|frame->data[4];
//		if(key<=5&&my_flag==0){
//			memcpy(&target_x[key-1],&frame->data[1],4);
//			memcpy(&target_y[key-1],&frame->data[5],4);
//			distance_to_angle(target_x[key-1],target_y[key-1],&angle_pitch[key-1],&angle_yaw[key-1]);
//			openmv_send_command(&cmd,1);
//		}		
		if(frame->data[0]!=my_flag){
				offset_x = frame->data[1]<<8|frame->data[2];
				offset_y = frame->data[3]<<8|frame->data[4];
				my_flag = frame->data[0];
		}
}

// 串口接收回调函数：处理数据并按字节分类
void Data_Handle1() {
    // 获取接收到的数据长度
    uint16_t data_len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (data_len <= 0) return;

    // 处理OpenMV协议数据
    static OpenMVFrame_RX_t openmv_frame;
    uint8_t openmv_parse_state = 0;
    uint8_t openmv_data_idx = 0;

    for (uint16_t i = 0; i < data_len; i++) {
        uint8_t byte = handle_Buff1[i];

        // OpenMV协议解析状态机
        switch (openmv_parse_state) {
            case 0: // 等待帧头
                if (byte == OPENMV_FRAME_HEADER) {
                    openmv_frame.header = OPENMV_FRAME_HEADER;
                    openmv_parse_state = 1;
                }
                break;

            case 1: // 接收数据
                if (openmv_data_idx < 5) {
                    openmv_frame.data[openmv_data_idx++] = byte;
                    if (openmv_data_idx >= 5) {
                        openmv_parse_state = 2;
												openmv_data_idx = 0;
                    }
                }
                break;

            case 2: // 接收帧尾
                if (byte == OPENMV_FRAME_TAIL) {
                    openmv_frame.tail = OPENMV_FRAME_TAIL;
                    parse_openmv_frame(&openmv_frame); // 解析并分类数据
										openmv_parse_state = 0; // 重置状态机
							  }
                break;
        }

    }
}

// 发送数据到OpenMV
uint8_t send_status = 1;
void openmv_send_command(const uint8_t* data, uint8_t len) {
    if (len > OPENMV_MAX_TX_DATA_LEN) return;
    
    OpenMVFrame_TX_t frame;
    frame.header = OPENMV_FRAME_HEADER;
		frame.status = send_status;
		send_status++;
    memcpy(frame.data, data, len);
    frame.tail = OPENMV_FRAME_TAIL;
		UART_Transmit_DMA_Safe((uint8_t *)&frame,len+3);
}




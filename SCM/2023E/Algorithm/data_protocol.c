#include "data_protocol.h"


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
float target_x[5],target_y[5];
float angle_pitch[5],angle_yaw[5];
int key;
// 解析完整的OpenMV帧
static void parse_openmv_frame(OpenMVFrame_RX_t* frame) {
		my_flag = frame->data[0];
//				targetx = frame->data[1]<<8|frame->data[2];
//				targety = frame->data[3]<<8|frame->data[4];
		if(key<=5){
			memcpy(&target_x[key-1],&frame->data[1],4);
			memcpy(&target_y[key-1],&frame->data[5],4);
			distance_to_angle(target_x[key-1],target_y[key-1],&angle_pitch[key-1],&angle_yaw[key-1]);
		}			
}

// 串口接收回调函数：处理数据并按字节分类
void Data_Handle1() {
    // 获取接收到的数据长度
    uint16_t data_len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (data_len <= 0) return;

    // 处理OpenMV协议数据
    static OpenMVFrame_RX_t openmv_frame;
    static uint8_t openmv_parse_state = 0;
    static uint8_t openmv_data_idx = 0;

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
                if (openmv_data_idx < 9) {
                    openmv_frame.data[openmv_data_idx++] = byte;
                    if (openmv_data_idx >= 9) {
                        openmv_parse_state = 2;
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
void openmv_send_command(const uint8_t* data, uint8_t len) {
    if (len > OPENMV_MAX_TX_DATA_LEN) return;
    
    OpenMVFrame_TX_t frame;
    frame.header = OPENMV_FRAME_HEADER;
    memcpy(frame.data, data, len);
    frame.tail = OPENMV_FRAME_TAIL;
		UART_Transmit_DMA_Safe((uint8_t *)&frame,len+2);
}



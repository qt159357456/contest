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


// 将openmv_frame.data按字节分类到结构体
static void classify_data_by_bytes(const uint8_t* data, uint8_t len, OpenMVDataBytes_t* classified) {
    if (!data || !classified || len == 0) return;

    // 备份原始数据
    classified->data_len = len;
    memcpy(classified->raw_data, data, len);  // 复制原始数据到缓冲区

    // 1字节数据解析（取前4字节，不足则用0填充）
    classified->byte1.cmd_type = (len > 0) ? data[0] : 0;
    classified->byte1.target_count = (len > 1) ? data[1] : 0;
    classified->byte1.reserved[0] = (len > 2) ? data[2] : 0;
    classified->byte1.reserved[1] = (len > 3) ? data[3] : 0;

    // 2字节数据解析（小端模式，取第4-11字节）
    if (len > 4) {
        classified->byte2.x_coord = (data[5] << 8) | data[4];  // 高字节<<8 | 低字节
    } else {
        classified->byte2.x_coord = 0;
    }
    if (len > 6) {
        classified->byte2.y_coord = (data[7] << 8) | data[6];
    } else {
        classified->byte2.y_coord = 0;
    }
    if (len > 8) {
        classified->byte2.width = (data[9] << 8) | data[8];
    } else {
        classified->byte2.width = 0;
    }
    if (len > 10) {
        classified->byte2.height = (data[11] << 8) | data[10];
    } else {
        classified->byte2.height = 0;
    }

    // 4字节数据解析（小端模式，取第12-19字节）
    if (len > 12) {
        // 时间戳（uint32_t）：data[12]为低字节，data[15]为高字节
        classified->byte4.timestamp = (data[15] << 24) | (data[14] << 16) | 
                                     (data[13] << 8) | data[12];
    } else {
        classified->byte4.timestamp = 0;
    }
    if (len > 16) {
        // 置信度（float）：通过指针转换（注意小端模式兼容性）
        uint32_t float_raw = (data[19] << 24) | (data[18] << 16) | 
                            (data[17] << 8) | data[16];
        memcpy(&classified->byte4.confidence, &float_raw, sizeof(float));
    } else {
        classified->byte4.confidence = 0.0f;
    }
}

// 解析完整的OpenMV帧
static void parse_openmv_frame(OpenMVFrame_t* frame) {
    if (!frame) return;

    // 创建按字节分类的结构体实例
    OpenMVDataBytes_t classified_data;
    classify_data_by_bytes(frame->data, frame->len, &classified_data);

    // 示例：使用分类后的数据（根据命令类型处理）
//    switch (classified_data.byte1.cmd_type) {
//        case 0x01:  // 颜色块检测命令
//            debug_printf("检测到 %d 个目标：\r\n", classified_data.byte1.target_count);
//            debug_printf("坐标：(%d, %d)，大小：%dx%d\r\n", 
//                       classified_data.byte2.x_coord,
//                       classified_data.byte2.y_coord,
//                       classified_data.byte2.width,
//                       classified_data.byte2.height);
//            debug_printf("置信度：%.2f，时间戳：%lu\r\n",
//                       classified_data.byte4.confidence,
//                       classified_data.byte4.timestamp);
//            break;

//        case 0x02:  // 无目标命令
//            debug_printf("未检测到目标\r\n");
//            break;

//        default:
//            debug_printf("未知命令类型：0x%02X\r\n", classified_data.byte1.cmd_type);
//            break;
//    }
}


// 串口接收回调函数：处理数据并按字节分类
void Data_Handle1() {
    // 获取接收到的数据长度
    uint16_t data_len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (data_len <= 0) return;

    // 处理OpenMV协议数据
    static OpenMVFrame_t openmv_frame;
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

            case 1: // 接收命令字
                openmv_frame.cmd = byte;
                openmv_parse_state = 2;
                break;

            case 2: // 接收数据长度
                openmv_frame.len = byte;
                if (openmv_frame.len > OPENMV_MAX_DATA_LEN) {
                    openmv_parse_state = 0; // 数据过长，重置
                } else {
                    openmv_data_idx = 0;
                    openmv_parse_state = 3;
                }
                break;

            case 3: // 接收数据
                if (openmv_data_idx < openmv_frame.len) {
                    openmv_frame.data[openmv_data_idx++] = byte;
                    if (openmv_data_idx >= openmv_frame.len) {
                        openmv_parse_state = 4;
                    }
                }
                break;

            case 4: // 接收帧尾
                if (byte == OPENMV_FRAME_TAIL) {
                    openmv_frame.tail = OPENMV_FRAME_TAIL;
                    parse_openmv_frame(&openmv_frame); // 解析并分类数据
                }
                openmv_parse_state = 0; // 重置状态机
                break;
        }
    }
}

// 发送数据到OpenMV
void openmv_send_command(uint8_t cmd, const uint8_t* data, uint8_t len) {
    if (len > OPENMV_MAX_DATA_LEN) return;
    
    OpenMVFrame_t frame;
    frame.header = OPENMV_FRAME_HEADER;
    frame.cmd = cmd;
    frame.len = len;
    memcpy(frame.data, data, len);
    frame.tail = OPENMV_FRAME_TAIL;
		UART_Transmit_DMA_Safe((uint8_t *)&frame,len);
}

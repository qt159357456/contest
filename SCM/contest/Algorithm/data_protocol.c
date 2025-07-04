#include "data_protocol.h"


// ���ڽ��ջ�����
uint8_t openmv_rx_buf[128] = {0};
uint16_t openmv_rx_len = 0;
// OpenMV���ݼ�������
//OpenMVData_t g_openmv_data = {TARGET_NONE};
CustomMutex_t openmv_data_mutex = {
    .locked = 0,
    .owner = NULL,
    .name = "openmv_data_mutex",
    .lock_count = 0
};


// ��openmv_frame.data���ֽڷ��ൽ�ṹ��
static void classify_data_by_bytes(const uint8_t* data, uint8_t len, OpenMVDataBytes_t* classified) {
    if (!data || !classified || len == 0) return;

    // ����ԭʼ����
    classified->data_len = len;
    memcpy(classified->raw_data, data, len);  // ����ԭʼ���ݵ�������

    // 1�ֽ����ݽ�����ȡǰ4�ֽڣ���������0��䣩
    classified->byte1.cmd_type = (len > 0) ? data[0] : 0;
    classified->byte1.target_count = (len > 1) ? data[1] : 0;
    classified->byte1.reserved[0] = (len > 2) ? data[2] : 0;
    classified->byte1.reserved[1] = (len > 3) ? data[3] : 0;

    // 2�ֽ����ݽ�����С��ģʽ��ȡ��4-11�ֽڣ�
    if (len > 4) {
        classified->byte2.x_coord = (data[5] << 8) | data[4];  // ���ֽ�<<8 | ���ֽ�
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

    // 4�ֽ����ݽ�����С��ģʽ��ȡ��12-19�ֽڣ�
    if (len > 12) {
        // ʱ�����uint32_t����data[12]Ϊ���ֽڣ�data[15]Ϊ���ֽ�
        classified->byte4.timestamp = (data[15] << 24) | (data[14] << 16) | 
                                     (data[13] << 8) | data[12];
    } else {
        classified->byte4.timestamp = 0;
    }
    if (len > 16) {
        // ���Ŷȣ�float����ͨ��ָ��ת����ע��С��ģʽ�����ԣ�
        uint32_t float_raw = (data[19] << 24) | (data[18] << 16) | 
                            (data[17] << 8) | data[16];
        memcpy(&classified->byte4.confidence, &float_raw, sizeof(float));
    } else {
        classified->byte4.confidence = 0.0f;
    }
}

// ����������OpenMV֡
static void parse_openmv_frame(OpenMVFrame_t* frame) {
    if (!frame) return;

    // �������ֽڷ���Ľṹ��ʵ��
    OpenMVDataBytes_t classified_data;
    classify_data_by_bytes(frame->data, frame->len, &classified_data);

    // ʾ����ʹ�÷��������ݣ������������ʹ���
//    switch (classified_data.byte1.cmd_type) {
//        case 0x01:  // ��ɫ��������
//            debug_printf("��⵽ %d ��Ŀ�꣺\r\n", classified_data.byte1.target_count);
//            debug_printf("���꣺(%d, %d)����С��%dx%d\r\n", 
//                       classified_data.byte2.x_coord,
//                       classified_data.byte2.y_coord,
//                       classified_data.byte2.width,
//                       classified_data.byte2.height);
//            debug_printf("���Ŷȣ�%.2f��ʱ�����%lu\r\n",
//                       classified_data.byte4.confidence,
//                       classified_data.byte4.timestamp);
//            break;

//        case 0x02:  // ��Ŀ������
//            debug_printf("δ��⵽Ŀ��\r\n");
//            break;

//        default:
//            debug_printf("δ֪�������ͣ�0x%02X\r\n", classified_data.byte1.cmd_type);
//            break;
//    }
}


// ���ڽ��ջص��������������ݲ����ֽڷ���
void Data_Handle1() {
    // ��ȡ���յ������ݳ���
    uint16_t data_len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    if (data_len <= 0) return;

    // ����OpenMVЭ������
    static OpenMVFrame_t openmv_frame;
    static uint8_t openmv_parse_state = 0;
    static uint8_t openmv_data_idx = 0;

    for (uint16_t i = 0; i < data_len; i++) {
        uint8_t byte = handle_Buff1[i];

        // OpenMVЭ�����״̬��
        switch (openmv_parse_state) {
            case 0: // �ȴ�֡ͷ
                if (byte == OPENMV_FRAME_HEADER) {
                    openmv_frame.header = OPENMV_FRAME_HEADER;
                    openmv_parse_state = 1;
                }
                break;

            case 1: // ����������
                openmv_frame.cmd = byte;
                openmv_parse_state = 2;
                break;

            case 2: // �������ݳ���
                openmv_frame.len = byte;
                if (openmv_frame.len > OPENMV_MAX_DATA_LEN) {
                    openmv_parse_state = 0; // ���ݹ���������
                } else {
                    openmv_data_idx = 0;
                    openmv_parse_state = 3;
                }
                break;

            case 3: // ��������
                if (openmv_data_idx < openmv_frame.len) {
                    openmv_frame.data[openmv_data_idx++] = byte;
                    if (openmv_data_idx >= openmv_frame.len) {
                        openmv_parse_state = 4;
                    }
                }
                break;

            case 4: // ����֡β
                if (byte == OPENMV_FRAME_TAIL) {
                    openmv_frame.tail = OPENMV_FRAME_TAIL;
                    parse_openmv_frame(&openmv_frame); // ��������������
                }
                openmv_parse_state = 0; // ����״̬��
                break;
        }
    }
}

// �������ݵ�OpenMV
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

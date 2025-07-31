#ifndef 	GLOBAL_H
#define   GLOBAL_H
#include "stdint.h"
#include "math.h"
//#include "myMath.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "stdlib.h"

typedef uint8_t u8;

// ����������������ʵ��Ӳ���޸ģ���λ������/ת��
#define ENCODER_PPR 1120  // ���������ÿת1120�����壨��������Ӳ����д��
#define CONTROL_PERIOD 0.01f  // ��������10ms�����������ȡ����һ�£�

typedef enum {
    STATE_INIT,          // ��ʼ��״̬
    STATE_MOVING,        // �ƶ�״̬
    STATE_ARRIVED,       // ����·����״̬
    STATE_FINISHED,      // ���·��״̬
		STATE_TURNING_IN_PLACE, // ԭ����ת״̬
    STATE_ERROR          // ����״̬
} NavState_t;

// ��Ϣ����״̬ö��
typedef enum {
    QUEUE_OK = 0,
    QUEUE_FULL,
    QUEUE_EMPTY,
    QUEUE_ERROR,
    QUEUE_TIMEOUT
} QueueStatus_t;

typedef struct {
    float Target;      // Ŀ��ֵ���趨�㣩
    float Actual;      // ��ǰʵ��ֵ������ֵ��
    float Actual1;     // ��һ�ε�ʵ��ֵ
    float Out;         // PID ���ֵ
    
    float Kp;          // ����ϵ��
    float Ki;          // ����ϵ��
    float Kd;          // ΢��ϵ��
    
    float Error0;      // ��ǰ��� (e[k])
    float Error1;      // ��һ����� (e[k-1])
    float ErrorInt;    // ������ֵ
    
    float ErrorIntMax; // ���������ޣ������ֱ��ͣ�
    float ErrorIntMin; // ����������
    float OutMax;      // �������
    float OutMin;      // �������
	
		float pOut;
		float iOut;
		float dOut;
		float TargetChangeThreshold;
		
		float IntMinThreshold;
		
} PID_t;


typedef struct {
	float Alpha;
	double Pre_out;
	double Pre_in;
}Filter_t;

#define FILTER_DEPTH 10 // �˲�����ȣ���������С��
//������ֵ�˲�
typedef struct {
    float *buffer; // ���ݻ�����ָ��
    int *index;    // ��ǰд��λ������
    float *sum;    // ��ǰ�ܺ�ָ��
    int size;      // ����������
}AVG_Flt_t;

//��ֵ�˲�
#define MEDIAN_LEN 10
typedef struct {
	double Data[MEDIAN_LEN];
	uint8_t index;
}MEDIAN_Flt_t;

////��ͨ�˲�
//#define N 101
//#define PI 3.1415926535897932384626
//typedef struct {
//	float Fs;
//	float Fc1;
//	float Fc2;
//	float b[N];
//	float buffer[N];
//	int index;
//}FIR_Flt_t;

//�������˲�
#define PI 3.1415926535897932384626
typedef struct{
		float LastP;   // ����������Э���� (a posteriori error covariance)
		float NewP;    // ����������Э���� (a priori error covariance)
    float Q;       // ��������Э���� (process noise covariance)
    float R;       // ��������Э���� (measurement noise covariance)
    float Kg;      // ���������� (Kalman gain)
    float Out;     // ���Ź������ֵ (optimal estimated output)
}Klm_Flt_t;


//����һ��������Ԫ���Ľṹ�����
typedef struct 
{
    float q0, q1, q2, q3;
} Quaternion;

/* �����ǻ������ */
typedef struct{
	float X;
	float Y;
	float Z;
}FLOAT_XYZ_t;


typedef struct{
	int16_t X;
	int16_t Y;
	int16_t Z;
}INT16_XYZ_t;

typedef struct {
    float pitch;   // �����ǣ���Y����ת��
    float roll;    // ����ǣ���X����ת��
    float yaw;     // ƫ���ǣ���Z����ת��- ��Ȼδ�ڳ�ʼ������ʾ����ͨ������
} Angle_t;


typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} MPU6050_Data_t;


typedef struct
{
	float x1; //
	float x2;
	float x3;
	float x;
	float r;
	float h;
	float T;
	float aim;
} TD;

typedef struct {
    float base_speed;    // �������ٶ� (m/s)
    float target_angle;  // Ŀ��ƫ���� (��)
} NavCmd_t;

// ������״̬�ṹ�壨ÿ��������������
typedef struct {
    int32_t last_count;      // �ϴμ���ֵ
    uint32_t last_tick;      // �ϴ�ʱ���
    int32_t total_pulses;    // �ۼ�����������ֹ�����
} EncoderState;

// ȫ������ϵ״̬
typedef struct {
    float x;        // X���� (m)
    float y;        // Y���� (m)
    float yaw;      // ƫ���� (��)
    float vx;       // X�����ٶ� (m/s)
    float vy;       // Y�����ٶ� (m/s)
} GlobalState_t;

// �ֲ�����ϵ�ٶ�
typedef struct {
    float linear;   // ǰ���ٶ� (m/s)
    float angular;  // ��ת���ٶ� (rad/s)
} LocalVelocity_t;

// ������״̬�ṹ��
typedef struct {
    GlobalState_t global;        // ȫ��״̬
    LocalVelocity_t local_vel;   // �ֲ��ٶ�
    
    // ����״̬
    float target_speed;          // Ŀ���ٶ� (m/s)
    float target_yaw;            // Ŀ��ƫ���� (��)
    int current_waypoint;        // ��ǰ·��������
    
    // ϵͳ״̬
    uint32_t update_count;       // ״̬���¼���
		NavState_t nav_state;        // ����״̬
	
		float left_speed;
		float right_speed;
	

} RobotState_t;

// ����ϵ�ṹ��
typedef struct {
    float x;  // X���� (m)
    float y;  // Y���� (m)
} Point2D_t;

typedef struct{
		float pitch;
		float yaw;
}Angles_t;

// �Զ��廥�����ṹ��
typedef struct {
    volatile uint8_t locked;          // ��״̬��0-δ������1-������
    TaskHandle_t owner;               // ��������������
    const char* name;                 // �����ƣ����ڵ���
    volatile uint32_t lock_count;     // ���������֧�ֵݹ�����
} CustomMutex_t;

// �Զ�����Ϣ���нṹ��
typedef struct {
    void* buffer;               // ��Ϣ������
    uint16_t msg_size;          // ������Ϣ��С���ֽڣ�
    uint16_t max_items;         // �����Ϣ����
    volatile uint16_t item_count; // ��ǰ��Ϣ����
    volatile uint16_t read_idx;  // ��ָ��
    volatile uint16_t write_idx; // дָ��
    CustomMutex_t mutex;        // ���в���������
    const char* name;           // �������ƣ������ã�
} CustomQueue_t;

// OpenMV��⵽��Ŀ������
//typedef enum {
//    TARGET_NONE = 0,
//    TARGET_COLOR_BLOCK,    // ��ɫ��
//    TARGET_LINE,           // ����
//    TARGET_FACE,           // ����
//    TARGET_QRCODE          // ��ά��
//} TargetType_t;

//// ��ɫʶ��ṹ��
//typedef struct {
//    uint16_t x;            // ����X����
//    uint16_t y;            // ����Y����
//    uint16_t width;        // ���
//    uint16_t height;       // �߶�
//    uint8_t color;         // ��ɫ����(0-��,1-��,2-����)
//    uint8_t confidence;    // ���Ŷ�(0-100)
//} ColorBlock_t;

// OpenMV������ݽṹ��
typedef struct {
    uint8_t target_count;  // Ŀ������
    uint32_t timestamp;    // ʱ���
    uint8_t valid;         // ������Ч�Ա�־(1-��Ч,0-��Ч)
		float cv;
} OpenMVData_t;

// ͨ��Э��֡�ṹ
#define OPENMV_FRAME_HEADER 0x55
#define OPENMV_FRAME_TAIL 0x0D
#define OPENMV_MAX_TX_DATA_LEN 1
#define OPENMV_MAX_RX_DATA_LEN 17

typedef struct {
    uint8_t header;        // ֡ͷ
		uint8_t status;
    uint8_t data[OPENMV_MAX_TX_DATA_LEN]; // ������
    uint8_t tail;          // ֡β
} OpenMVFrame_TX_t;

typedef struct {
    uint8_t header;        // ֡ͷ
    uint8_t data[OPENMV_MAX_RX_DATA_LEN]; // ������
    uint8_t tail;          // ֡β
} OpenMVFrame_RX_t;

// ���ֽڳ��ȷ�������ݽṹ��
typedef struct {
    // 1�ֽ����ݣ�uint8_t��
    struct {
        uint8_t cmd_type;       // �������ͣ���0�ֽڣ�
        uint8_t target_count;   // Ŀ����������1�ֽڣ�
        uint8_t reserved[2];    // Ԥ���ֽڣ���2-3�ֽڣ�
    } byte1;

    // 2�ֽ����ݣ�uint16_t��С��ģʽ��
    struct {
        uint16_t x_coord;       // X���꣨��4-5�ֽڣ�
        uint16_t y_coord;       // Y���꣨��6-7�ֽڣ�
        uint16_t width;         // ��ȣ���8-9�ֽڣ�
        uint16_t height;        // �߶ȣ���10-11�ֽڣ�
    } byte2;

    // 4�ֽ����ݣ�uint32_t/float��С��ģʽ��
    struct {
        uint32_t timestamp;     // ʱ�������12-15�ֽڣ�
        float confidence;       // ���Ŷȣ���16-19�ֽڣ�0.0-1.0��
    } byte4;

    // ԭʼ���ݻ��������������ݱ��ݣ�
    uint8_t raw_data[OPENMV_MAX_RX_DATA_LEN];
    uint8_t data_len;          // ʵ�����ݳ���
} OpenMVDataBytes_t;

// ѭ��״̬
typedef enum {
		STATE_STOPPED,      // ֹͣ״̬����ⲻ�����ߣ�
		STATE_STRAIGHT,     // ֱ��״̬
		STATE_LEFT_TURN,    // ��ת״̬
		STATE_RIGHT_TURN    // ��ת״̬
}State_t;

// ������Ϣ�ṹ��
typedef struct {
    float target_angle;
    float current_yaw;
    float angle_adjust;
    float base_speed;
    uint32_t timestamp;
} DebugInfo_t;




/* ����������ݽṹ�� */
typedef struct {
    /* ���״̬ */
    float current_angle;          // ��ǰ�Ƕ�λ�ã��ȣ�
    float target_angle;           // Ŀ��Ƕ�λ�ã��ȣ�
    uint32_t step_target;         // Ŀ�경��
    uint32_t step_count;          // ��ǰ���߲���
		int32_t step_sum;             // �ߵ��ܵ������� 
    uint8_t direction;            // ת������ (0: ����, 1: ����)
    
    /* Ӳ������ */
    GPIO_TypeDef* dir_port;       // �������GPIO�˿�
    uint16_t dir_pin;             // �����������
    TIM_HandleTypeDef* timer;     // ��ʱ�����
    uint32_t timer_channel;       // ��ʱ��ͨ��
    
    /* ������� */
    float step_angle;             // ����ǣ���/����
    uint32_t max_frequency;       // �������Ƶ�ʣ�Hz��
    
    /* ���Ʊ�־ */
    uint8_t is_moving;            // �˶�״̬��־ (1: �˶���, 0: ֹͣ)
} StepperMotor;



#define LEFT_MOTOR_PWM_TIMER        htim1
#define LEFT_MOTOR_PWM_CHANNEL      TIM_CHANNEL_1
#define RIGHT_MOTOR_PWM_TIMER       htim1
#define RIGHT_MOTOR_PWM_CHANNEL     TIM_CHANNEL_2
#define LEFT_MOTOR_ENCODER_TIMER    htim3
#define RIGHT_MOTOR_ENCODER_TIMER   htim2

#define TIME_WAIT 20    // �ٶȼ���������룩
#define PULSES_NUMBER (2 * 6.3f * 11) // ������ÿת������
#define WHEEL_DIAMETER 0.065f // 65mm����
#define WHEELBASE 0.12f      // ǰ������� (��λ����)
#define TRACK_WIDTH 0.18f    // �����־� (��λ����)

#define NAV_QUEUE_SIZE 5

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128
#define OPENMV_RX_BUFFER_SIZE 128
#define OPENMV_TX_BUFFER_SIZE 3

// ѭ�����Ʋ���
#define STRAIGHT_SPEED 0.5f    // ����ǰ���ٶ� (m/s)
#define TURN_SPEED 0.2f    // ����ٶ� (m/s)
#define MAX_ANGLE_ADJ 60.0f   // ���Ƕȵ�����(��)
#define LOST_THRESHOLD 15      // ��ʧ��·������ֵ(Լ150ms)
#define SMALL_ADJUST 10.0f;     // С�Ƕȵ�����(��)
#define LARGE_ADJUST 25.0f;     // ��Ƕȵ�����(��)

#define squa( Sq )        (((float)Sq)*((float)Sq))
	





#define SCREEN_DISTANCE 1000  // ��̨����Ļ����1000mm
#define SCREEN_WIDTH    500   // ��Ļ���500mm
#define STEP_ANGLE 0.05625f
#define INTERPOLATION_STEPS   20 //·�����ɶ���


/* �������� */
#define X_MOTOR_HUART huart2
#define Y_MOTOR_HUART huart4


#define CMD_ENABLE        0xF3
#define CMD_SPEED_MODE    0xF6
#define CMD_POSITION_MODE 0xFD
#define CMD_STOP          0xFE
#define CMD_SYNC_MOVE     0xFF
#define CMD_SET_HOME      0x93
#define CMD_TRIGGER_HOME  0x9A
#define CMD_ABORT_HOME    0x9C
#define CMD_READ_SPEED    0x35
#define CMD_READ_POSITION 0x36
/* ��������� */
#define DIR_CW   0x00
#define DIR_CCW  0x01

/* ���У���ֽ� */
#define CHECKSUM 0x6B

/* ������ݽṹ���� */
typedef struct {
    uint8_t speed_sign;     // ���� (0:��, 1:��)
    uint16_t speed;   // ת�� (RPM)
		float f_speed; //��/s
		uint8_t position_sign;     // ���� (0:��, 1:��)
    uint32_t position; // λ��ֵ (�������)
    float angle;      // �����ĽǶ�ֵ
} MotorData;


/********************************************************************************/
#endif


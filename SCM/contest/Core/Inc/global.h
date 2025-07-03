#ifndef 	GLOBAL_H
#define   GLOBAL_H
#include "stdint.h"
#include "math.h"
#include "myMath.h"

typedef uint8_t u8;

// ����������������ʵ��Ӳ���޸ģ���λ������/ת��
#define ENCODER_PPR 1120  // ���������ÿת1120�����壨��������Ӳ����д��
#define CONTROL_PERIOD 0.01f  // ��������10ms�����������ȡ����һ�£�


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
} RobotState_t;

// ����ϵ�ṹ��
typedef struct {
    float x;  // X���� (m)
    float y;  // Y���� (m)
} Point2D_t;

#define LEFT_MOTOR_PWM_TIMER        htim3
#define LEFT_MOTOR_PWM_CHANNEL      TIM_CHANNEL_4
#define RIGHT_MOTOR_PWM_TIMER       htim3
#define RIGHT_MOTOR_PWM_CHANNEL     TIM_CHANNEL_3
#define LEFT_MOTOR_ENCODER_TIMER    htim3
#define RIGHT_MOTOR_ENCODER_TIMER   htim2

#define TIME_WAIT 20    // �ٶȼ���������룩
#define PULSES_NUMBER (2 * 6.3f * 11) // ������ÿת������
#define WHEEL_DIAMETER 0.065f // 65mm����
#define WHEELBASE 0.25f      // ǰ������� (��λ����)
#define TRACK_WIDTH 0.20f    // �����־� (��λ����)




#define squa( Sq )        (((float)Sq)*((float)Sq))
#endif


























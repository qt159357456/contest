#ifndef 	GLOBAL_H
#define   GLOBAL_H
#include "stdint.h"
#include "math.h"
#include "myMath.h"

typedef uint8_t u8;

// 编码器参数（根据实际硬件修改，单位：脉冲/转）
#define ENCODER_PPR 1120  // 假设编码器每转1120个脉冲（需根据你的硬件填写）
#define CONTROL_PERIOD 0.01f  // 控制周期10ms（与编码器读取周期一致）


typedef struct {
    float Target;      // 目标值（设定点）
    float Actual;      // 当前实际值（反馈值）
    float Actual1;     // 上一次的实际值
    float Out;         // PID 输出值
    
    float Kp;          // 比例系数
    float Ki;          // 积分系数
    float Kd;          // 微分系数
    
    float Error0;      // 当前误差 (e[k])
    float Error1;      // 上一次误差 (e[k-1])
    float ErrorInt;    // 误差积分值
    
    float ErrorIntMax; // 积分项上限（抗积分饱和）
    float ErrorIntMin; // 积分项下限
    float OutMax;      // 输出上限
    float OutMin;      // 输出下限
} PID_t;

typedef struct {
	float Alpha;
	double Pre_out;
	double Pre_in;
}Filter_t;

#define FILTER_DEPTH 10 // 滤波器深度（缓冲区大小）
//滑动均值滤波
typedef struct {
    float *buffer; // 数据缓冲区指针
    int *index;    // 当前写入位置索引
    float *sum;    // 当前总和指针
    int size;      // 缓冲区容量
}AVG_Flt_t;

//中值滤波
#define MEDIAN_LEN 10
typedef struct {
	double Data[MEDIAN_LEN];
	uint8_t index;
}MEDIAN_Flt_t;

////带通滤波
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

//卡尔曼滤波
#define PI 3.1415926535897932384626
typedef struct{
		float LastP;   // 后验估计误差协方差 (a posteriori error covariance)
		float NewP;    // 先验估计误差协方差 (a priori error covariance)
    float Q;       // 过程噪声协方差 (process noise covariance)
    float R;       // 测量噪声协方差 (measurement noise covariance)
    float Kg;      // 卡尔曼增益 (Kalman gain)
    float Out;     // 最优估计输出值 (optimal estimated output)
}Klm_Flt_t;


//定义一个关于四元数的结构体变量
typedef struct 
{
    float q0, q1, q2, q3;
} Quaternion;

/* 陀螺仪积分误差 */
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
    float pitch;   // 俯仰角（绕Y轴旋转）
    float roll;    // 横滚角（绕X轴旋转）
    float yaw;     // 偏航角（绕Z轴旋转）- 虽然未在初始化中显示，但通常包含
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
    float base_speed;    // 基础线速度 (m/s)
    float target_angle;  // 目标偏航角 (度)
} NavCmd_t;

// 编码器状态结构体（每个编码器独立）
typedef struct {
    int32_t last_count;      // 上次计数值
    uint32_t last_tick;      // 上次时间戳
    int32_t total_pulses;    // 累计脉冲数（防止溢出）
} EncoderState;

// 全局坐标系状态
typedef struct {
    float x;        // X坐标 (m)
    float y;        // Y坐标 (m)
    float yaw;      // 偏航角 (度)
    float vx;       // X方向速度 (m/s)
    float vy;       // Y方向速度 (m/s)
} GlobalState_t;

// 局部坐标系速度
typedef struct {
    float linear;   // 前进速度 (m/s)
    float angular;  // 旋转角速度 (rad/s)
} LocalVelocity_t;

// 机器人状态结构体
typedef struct {
    GlobalState_t global;        // 全局状态
    LocalVelocity_t local_vel;   // 局部速度
    
    // 控制状态
    float target_speed;          // 目标速度 (m/s)
    float target_yaw;            // 目标偏航角 (度)
    int current_waypoint;        // 当前路径点索引
    
    // 系统状态
    uint32_t update_count;       // 状态更新计数
} RobotState_t;

// 坐标系结构体
typedef struct {
    float x;  // X坐标 (m)
    float y;  // Y坐标 (m)
} Point2D_t;

#define LEFT_MOTOR_PWM_TIMER        htim3
#define LEFT_MOTOR_PWM_CHANNEL      TIM_CHANNEL_4
#define RIGHT_MOTOR_PWM_TIMER       htim3
#define RIGHT_MOTOR_PWM_CHANNEL     TIM_CHANNEL_3
#define LEFT_MOTOR_ENCODER_TIMER    htim3
#define RIGHT_MOTOR_ENCODER_TIMER   htim2

#define TIME_WAIT 20    // 速度检测间隔（毫秒）
#define PULSES_NUMBER (2 * 6.3f * 11) // 编码器每转脉冲数
#define WHEEL_DIAMETER 0.065f // 65mm轮子
#define WHEELBASE 0.25f      // 前后轮轴距 (单位：米)
#define TRACK_WIDTH 0.20f    // 后轮轮距 (单位：米)




#define squa( Sq )        (((float)Sq)*((float)Sq))
#endif


























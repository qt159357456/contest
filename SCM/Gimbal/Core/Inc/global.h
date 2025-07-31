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

// 编码器参数（根据实际硬件修改，单位：脉冲/转）
#define ENCODER_PPR 1120  // 假设编码器每转1120个脉冲（需根据你的硬件填写）
#define CONTROL_PERIOD 0.01f  // 控制周期10ms（与编码器读取周期一致）

typedef enum {
    STATE_INIT,          // 初始化状态
    STATE_MOVING,        // 移动状态
    STATE_ARRIVED,       // 到达路径点状态
    STATE_FINISHED,      // 完成路径状态
		STATE_TURNING_IN_PLACE, // 原地旋转状态
    STATE_ERROR          // 错误状态
} NavState_t;

// 消息队列状态枚举
typedef enum {
    QUEUE_OK = 0,
    QUEUE_FULL,
    QUEUE_EMPTY,
    QUEUE_ERROR,
    QUEUE_TIMEOUT
} QueueStatus_t;

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
		NavState_t nav_state;        // 导航状态
	
		float left_speed;
		float right_speed;
	

} RobotState_t;

// 坐标系结构体
typedef struct {
    float x;  // X坐标 (m)
    float y;  // Y坐标 (m)
} Point2D_t;

typedef struct{
		float pitch;
		float yaw;
}Angles_t;

// 自定义互斥锁结构体
typedef struct {
    volatile uint8_t locked;          // 锁状态：0-未锁定，1-已锁定
    TaskHandle_t owner;               // 持有锁的任务句柄
    const char* name;                 // 锁名称，用于调试
    volatile uint32_t lock_count;     // 重入计数（支持递归锁）
} CustomMutex_t;

// 自定义消息队列结构体
typedef struct {
    void* buffer;               // 消息缓冲区
    uint16_t msg_size;          // 单个消息大小（字节）
    uint16_t max_items;         // 最大消息数量
    volatile uint16_t item_count; // 当前消息数量
    volatile uint16_t read_idx;  // 读指针
    volatile uint16_t write_idx; // 写指针
    CustomMutex_t mutex;        // 队列操作互斥锁
    const char* name;           // 队列名称（调试用）
} CustomQueue_t;

// OpenMV检测到的目标类型
//typedef enum {
//    TARGET_NONE = 0,
//    TARGET_COLOR_BLOCK,    // 颜色块
//    TARGET_LINE,           // 线条
//    TARGET_FACE,           // 人脸
//    TARGET_QRCODE          // 二维码
//} TargetType_t;

//// 颜色识别结构体
//typedef struct {
//    uint16_t x;            // 中心X坐标
//    uint16_t y;            // 中心Y坐标
//    uint16_t width;        // 宽度
//    uint16_t height;       // 高度
//    uint8_t color;         // 颜色代码(0-红,1-绿,2-蓝等)
//    uint8_t confidence;    // 置信度(0-100)
//} ColorBlock_t;

// OpenMV检测数据结构体
typedef struct {
    uint8_t target_count;  // 目标数量
    uint32_t timestamp;    // 时间戳
    uint8_t valid;         // 数据有效性标志(1-有效,0-无效)
		float cv;
} OpenMVData_t;

// 通信协议帧结构
#define OPENMV_FRAME_HEADER 0x55
#define OPENMV_FRAME_TAIL 0x0D
#define OPENMV_MAX_TX_DATA_LEN 1
#define OPENMV_MAX_RX_DATA_LEN 17

typedef struct {
    uint8_t header;        // 帧头
		uint8_t status;
    uint8_t data[OPENMV_MAX_TX_DATA_LEN]; // 数据区
    uint8_t tail;          // 帧尾
} OpenMVFrame_TX_t;

typedef struct {
    uint8_t header;        // 帧头
    uint8_t data[OPENMV_MAX_RX_DATA_LEN]; // 数据区
    uint8_t tail;          // 帧尾
} OpenMVFrame_RX_t;

// 按字节长度分类的数据结构体
typedef struct {
    // 1字节数据（uint8_t）
    struct {
        uint8_t cmd_type;       // 命令类型（第0字节）
        uint8_t target_count;   // 目标数量（第1字节）
        uint8_t reserved[2];    // 预留字节（第2-3字节）
    } byte1;

    // 2字节数据（uint16_t，小端模式）
    struct {
        uint16_t x_coord;       // X坐标（第4-5字节）
        uint16_t y_coord;       // Y坐标（第6-7字节）
        uint16_t width;         // 宽度（第8-9字节）
        uint16_t height;        // 高度（第10-11字节）
    } byte2;

    // 4字节数据（uint32_t/float，小端模式）
    struct {
        uint32_t timestamp;     // 时间戳（第12-15字节）
        float confidence;       // 置信度（第16-19字节，0.0-1.0）
    } byte4;

    // 原始数据缓冲区（完整数据备份）
    uint8_t raw_data[OPENMV_MAX_RX_DATA_LEN];
    uint8_t data_len;          // 实际数据长度
} OpenMVDataBytes_t;

// 循迹状态
typedef enum {
		STATE_STOPPED,      // 停止状态（检测不到黑线）
		STATE_STRAIGHT,     // 直行状态
		STATE_LEFT_TURN,    // 左转状态
		STATE_RIGHT_TURN    // 右转状态
}State_t;

// 调试信息结构体
typedef struct {
    float target_angle;
    float current_yaw;
    float angle_adjust;
    float base_speed;
    uint32_t timestamp;
} DebugInfo_t;




/* 步进电机数据结构体 */
typedef struct {
    /* 电机状态 */
    float current_angle;          // 当前角度位置（度）
    float target_angle;           // 目标角度位置（度）
    uint32_t step_target;         // 目标步数
    uint32_t step_count;          // 当前已走步数
		int32_t step_sum;             // 走的总的脉冲数 
    uint8_t direction;            // 转动方向 (0: 正向, 1: 反向)
    
    /* 硬件配置 */
    GPIO_TypeDef* dir_port;       // 方向控制GPIO端口
    uint16_t dir_pin;             // 方向控制引脚
    TIM_HandleTypeDef* timer;     // 定时器句柄
    uint32_t timer_channel;       // 定时器通道
    
    /* 电机特性 */
    float step_angle;             // 步距角（度/步）
    uint32_t max_frequency;       // 最大驱动频率（Hz）
    
    /* 控制标志 */
    uint8_t is_moving;            // 运动状态标志 (1: 运动中, 0: 停止)
} StepperMotor;



#define LEFT_MOTOR_PWM_TIMER        htim1
#define LEFT_MOTOR_PWM_CHANNEL      TIM_CHANNEL_1
#define RIGHT_MOTOR_PWM_TIMER       htim1
#define RIGHT_MOTOR_PWM_CHANNEL     TIM_CHANNEL_2
#define LEFT_MOTOR_ENCODER_TIMER    htim3
#define RIGHT_MOTOR_ENCODER_TIMER   htim2

#define TIME_WAIT 20    // 速度检测间隔（毫秒）
#define PULSES_NUMBER (2 * 6.3f * 11) // 编码器每转脉冲数
#define WHEEL_DIAMETER 0.065f // 65mm轮子
#define WHEELBASE 0.12f      // 前后轮轴距 (单位：米)
#define TRACK_WIDTH 0.18f    // 后轮轮距 (单位：米)

#define NAV_QUEUE_SIZE 5

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128
#define OPENMV_RX_BUFFER_SIZE 128
#define OPENMV_TX_BUFFER_SIZE 3

// 循迹控制参数
#define STRAIGHT_SPEED 0.5f    // 基础前进速度 (m/s)
#define TURN_SPEED 0.2f    // 弯道速度 (m/s)
#define MAX_ANGLE_ADJ 60.0f   // 最大角度调整量(度)
#define LOST_THRESHOLD 15      // 丢失线路计数阈值(约150ms)
#define SMALL_ADJUST 10.0f;     // 小角度调整量(度)
#define LARGE_ADJUST 25.0f;     // 大角度调整量(度)

#define squa( Sq )        (((float)Sq)*((float)Sq))
	





#define SCREEN_DISTANCE 1000  // 云台到屏幕距离1000mm
#define SCREEN_WIDTH    500   // 屏幕宽度500mm
#define STEP_ANGLE 0.05625f
#define INTERPOLATION_STEPS   20 //路径生成段数


/* 电机命令定义 */
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
/* 电机方向定义 */
#define DIR_CW   0x00
#define DIR_CCW  0x01

/* 电机校验字节 */
#define CHECKSUM 0x6B

/* 电机数据结构定义 */
typedef struct {
    uint8_t speed_sign;     // 符号 (0:正, 1:负)
    uint16_t speed;   // 转速 (RPM)
		float f_speed; //度/s
		uint8_t position_sign;     // 符号 (0:正, 1:负)
    uint32_t position; // 位置值 (脉冲计数)
    float angle;      // 计算后的角度值
} MotorData;


/********************************************************************************/
#endif


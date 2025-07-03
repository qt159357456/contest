#include "main.h"
#include "motor.h"
#include "stdio.h"
#include "stdlib.h"
#include "global.h"
#include "filter.h"

// 全局编码器状态（左右电机各一个）
EncoderState left_encoder = {0};
EncoderState right_encoder = {0};

/**
 * @brief 初始化电机控制相关外设
 * @note 包括：
 *       1. 启动定时器3的PWM输出（通道3和4）
 *       2. 配置定时器2和4的编码器接口
 */
void motorInit(void)
{
    // 启动定时器1的PWM输出（通道3用于右电机，通道4用于左电机）
    HAL_TIM_PWM_Start(&RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL); // 右电机PWM通道
    HAL_TIM_PWM_Start(&LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL); // 左电机PWM通道

    // 配置定时器2为增量式编码器接口（A/B相正交信号）
    HAL_TIM_Encoder_Start(&RIGHT_MOTOR_ENCODER_TIMER, TIM_CHANNEL_ALL); // 右电机编码器

    // 配置定时器4为增量式编码器接口（A/B相正交信号）
    HAL_TIM_Encoder_Start(&LEFT_MOTOR_ENCODER_TIMER, TIM_CHANNEL_ALL); // 左电机编码器
}

/**
 * @brief 左电机控制函数
 * @param compare_value PWM占空比值（范围：-1000~1000）
 * @note  通过GPIO控制电机方向，通过定时器3通道4设置PWM占空比
 */
void leftMotorControl(int compare_value)
{
    // 方向控制逻辑（基于H桥电路）：
    if (compare_value > 0)
    {
        // 正转：IN1高电平，IN2低电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // IN2
    }
    else
    {
        // 反转：IN1低电平，IN2高电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // IN2
    }

    // 将占空比限制在有效范围内并设置PWM
    compare_value = abs(compare_value);                         // 取绝对值
    __HAL_TIM_SetCompare(&LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, compare_value); // 更新PWM占空比
}

/**
 * @brief 右电机控制函数
 * @param compare_value PWM占空比值（范围：-1000~1000）
 * @note  通过GPIO控制电机方向，通过定时器3通道3设置PWM占空比
 */
void rightMotorControl(int compare_value)
{
    // 方向控制逻辑（基于H桥电路）：
    if (compare_value > 0)
    {
        // 正转：IN1高电平，IN2低电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN2
    }
    else
    {
        // 反转：IN1低电平，IN2高电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // IN2
    }

    // 将占空比限制在有效范围内并设置PWM
    compare_value = abs(compare_value);                         // 取绝对值
    __HAL_TIM_SetCompare(&RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, compare_value); // 更新PWM占空比
}


// 全局滤波器缓冲区（左右电机各一个）
static float left_speed_buf[FILTER_DEPTH] = {0};  // 左电机速度缓冲区
static float right_speed_buf[FILTER_DEPTH] = {0}; // 右电机速度缓冲区
static int left_idx = 0;                          // 左电机缓冲区索引
static int right_idx = 0;                         // 右电机缓冲区索引
static float left_sum = 0.0f;                     // 左电机速度累计和
static float right_sum = 0.0f;                    // 右电机速度累计和

// 初始化环形缓冲区
static AVG_Flt_t left_rb = {left_speed_buf, &left_idx, &left_sum, FILTER_DEPTH};
static AVG_Flt_t right_rb = {right_speed_buf, &right_idx, &right_sum, FILTER_DEPTH};


/**
 * @brief 获取编码器计数值并重置计数器
 * @param htim 定时器句柄指针
 * @return 当前计数值（已转换为脉冲数）
 */
float encodeGetCount(TIM_HandleTypeDef *htim)
{
    // 获取当前计数值（自动转换为有符号整数）
    float count = (int16_t)__HAL_TIM_GET_COUNTER(htim);

    // 重置计数器（清零）
    __HAL_TIM_SET_COUNTER(htim, 0);

    return count;
}

/**
 * @brief 获取编码器相对计数值
 * @param htim 定时器句柄指针
 * @param state 编码器状态结构体
 * @return 自上次读取以来的脉冲增量
 */
int32_t getEncoderDelta(TIM_HandleTypeDef *htim, EncoderState *state) {
    // 读取当前计数值（有符号16位）
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    
    // 计算相对增量（处理16位溢出）
    int32_t delta = (int32_t)current_count - state->last_count;
    
    // 处理计数器溢出（0xFFFF -> 0x0000 或 0x0000 -> 0xFFFF）
    if (delta > 32767) {         // 正向溢出
        delta -= 65536;
    } else if (delta < -32768) { // 负向溢出
        delta += 65536;
    }
    
    // 更新状态
    state->last_count = current_count;
    state->total_pulses += delta;  // 累计总脉冲
    
    return delta;
}

/**
 * @brief 获取轮子线速度
 * @param htim 定时器句柄指针
 * @param state 编码器状态结构体
 * @return 线速度 (m/s)
 */
float getWheelSpeed(TIM_HandleTypeDef *htim, EncoderState *state) {
    // 获取当前时间
    uint32_t current_tick = uwTick;
    
    // 计算时间差 (秒)
    float time_diff = (current_tick - state->last_tick) * 0.001f;
    state->last_tick = current_tick;
    
    // 获取脉冲增量
    int32_t pulse_diff = getEncoderDelta(htim, state);
    
    // 计算线速度 (m/s)
    // 轮子线速度 = (脉冲增量/总脉冲) × 轮周长 ÷ 时间
    float speed = (pulse_diff / (float)PULSES_NUMBER) * (PI * WHEEL_DIAMETER) / time_diff;
    
    return speed;
}

// 左/右轮速获取
float leftWheelSpeed(void) { return vectorFilter(getWheelSpeed(&RIGHT_MOTOR_ENCODER_TIMER,&left_encoder), &left_rb); }
float rightWheelSpeed(void) { return vectorFilter(getWheelSpeed(&LEFT_MOTOR_ENCODER_TIMER,&right_encoder), &right_rb); }






/**
 * @brief 电机控制测试函数
 */
void motorTest(void)
{
    motorInit();            // 初始化电机系统
//    leftMotorControl(500);  // 左电机50%占空比
//    rightMotorControl(500); // 右电机50%占空比
}


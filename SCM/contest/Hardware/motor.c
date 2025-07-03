#include "main.h"
#include "motor.h"
#include "stdio.h"
#include "stdlib.h"
#include "global.h"
#include "filter.h"

// ȫ�ֱ�����״̬�����ҵ����һ����
EncoderState left_encoder = {0};
EncoderState right_encoder = {0};

/**
 * @brief ��ʼ����������������
 * @note ������
 *       1. ������ʱ��3��PWM�����ͨ��3��4��
 *       2. ���ö�ʱ��2��4�ı������ӿ�
 */
void motorInit(void)
{
    // ������ʱ��1��PWM�����ͨ��3�����ҵ����ͨ��4����������
    HAL_TIM_PWM_Start(&RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL); // �ҵ��PWMͨ��
    HAL_TIM_PWM_Start(&LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL); // ����PWMͨ��

    // ���ö�ʱ��2Ϊ����ʽ�������ӿڣ�A/B�������źţ�
    HAL_TIM_Encoder_Start(&RIGHT_MOTOR_ENCODER_TIMER, TIM_CHANNEL_ALL); // �ҵ��������

    // ���ö�ʱ��4Ϊ����ʽ�������ӿڣ�A/B�������źţ�
    HAL_TIM_Encoder_Start(&LEFT_MOTOR_ENCODER_TIMER, TIM_CHANNEL_ALL); // ����������
}

/**
 * @brief �������ƺ���
 * @param compare_value PWMռ�ձ�ֵ����Χ��-1000~1000��
 * @note  ͨ��GPIO���Ƶ������ͨ����ʱ��3ͨ��4����PWMռ�ձ�
 */
void leftMotorControl(int compare_value)
{
    // ��������߼�������H�ŵ�·����
    if (compare_value > 0)
    {
        // ��ת��IN1�ߵ�ƽ��IN2�͵�ƽ
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // IN2
    }
    else
    {
        // ��ת��IN1�͵�ƽ��IN2�ߵ�ƽ
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // IN2
    }

    // ��ռ�ձ���������Ч��Χ�ڲ�����PWM
    compare_value = abs(compare_value);                         // ȡ����ֵ
    __HAL_TIM_SetCompare(&LEFT_MOTOR_PWM_TIMER, LEFT_MOTOR_PWM_CHANNEL, compare_value); // ����PWMռ�ձ�
}

/**
 * @brief �ҵ�����ƺ���
 * @param compare_value PWMռ�ձ�ֵ����Χ��-1000~1000��
 * @note  ͨ��GPIO���Ƶ������ͨ����ʱ��3ͨ��3����PWMռ�ձ�
 */
void rightMotorControl(int compare_value)
{
    // ��������߼�������H�ŵ�·����
    if (compare_value > 0)
    {
        // ��ת��IN1�ߵ�ƽ��IN2�͵�ƽ
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN2
    }
    else
    {
        // ��ת��IN1�͵�ƽ��IN2�ߵ�ƽ
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // IN2
    }

    // ��ռ�ձ���������Ч��Χ�ڲ�����PWM
    compare_value = abs(compare_value);                         // ȡ����ֵ
    __HAL_TIM_SetCompare(&RIGHT_MOTOR_PWM_TIMER, RIGHT_MOTOR_PWM_CHANNEL, compare_value); // ����PWMռ�ձ�
}


// ȫ���˲��������������ҵ����һ����
static float left_speed_buf[FILTER_DEPTH] = {0};  // �����ٶȻ�����
static float right_speed_buf[FILTER_DEPTH] = {0}; // �ҵ���ٶȻ�����
static int left_idx = 0;                          // ��������������
static int right_idx = 0;                         // �ҵ������������
static float left_sum = 0.0f;                     // �����ٶ��ۼƺ�
static float right_sum = 0.0f;                    // �ҵ���ٶ��ۼƺ�

// ��ʼ�����λ�����
static AVG_Flt_t left_rb = {left_speed_buf, &left_idx, &left_sum, FILTER_DEPTH};
static AVG_Flt_t right_rb = {right_speed_buf, &right_idx, &right_sum, FILTER_DEPTH};


/**
 * @brief ��ȡ����������ֵ�����ü�����
 * @param htim ��ʱ�����ָ��
 * @return ��ǰ����ֵ����ת��Ϊ��������
 */
float encodeGetCount(TIM_HandleTypeDef *htim)
{
    // ��ȡ��ǰ����ֵ���Զ�ת��Ϊ�з���������
    float count = (int16_t)__HAL_TIM_GET_COUNTER(htim);

    // ���ü����������㣩
    __HAL_TIM_SET_COUNTER(htim, 0);

    return count;
}

/**
 * @brief ��ȡ��������Լ���ֵ
 * @param htim ��ʱ�����ָ��
 * @param state ������״̬�ṹ��
 * @return ���ϴζ�ȡ��������������
 */
int32_t getEncoderDelta(TIM_HandleTypeDef *htim, EncoderState *state) {
    // ��ȡ��ǰ����ֵ���з���16λ��
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    
    // �����������������16λ�����
    int32_t delta = (int32_t)current_count - state->last_count;
    
    // ��������������0xFFFF -> 0x0000 �� 0x0000 -> 0xFFFF��
    if (delta > 32767) {         // �������
        delta -= 65536;
    } else if (delta < -32768) { // �������
        delta += 65536;
    }
    
    // ����״̬
    state->last_count = current_count;
    state->total_pulses += delta;  // �ۼ�������
    
    return delta;
}

/**
 * @brief ��ȡ�������ٶ�
 * @param htim ��ʱ�����ָ��
 * @param state ������״̬�ṹ��
 * @return ���ٶ� (m/s)
 */
float getWheelSpeed(TIM_HandleTypeDef *htim, EncoderState *state) {
    // ��ȡ��ǰʱ��
    uint32_t current_tick = uwTick;
    
    // ����ʱ��� (��)
    float time_diff = (current_tick - state->last_tick) * 0.001f;
    state->last_tick = current_tick;
    
    // ��ȡ��������
    int32_t pulse_diff = getEncoderDelta(htim, state);
    
    // �������ٶ� (m/s)
    // �������ٶ� = (��������/������) �� ���ܳ� �� ʱ��
    float speed = (pulse_diff / (float)PULSES_NUMBER) * (PI * WHEEL_DIAMETER) / time_diff;
    
    return speed;
}

// ��/�����ٻ�ȡ
float leftWheelSpeed(void) { return vectorFilter(getWheelSpeed(&RIGHT_MOTOR_ENCODER_TIMER,&left_encoder), &left_rb); }
float rightWheelSpeed(void) { return vectorFilter(getWheelSpeed(&LEFT_MOTOR_ENCODER_TIMER,&right_encoder), &right_rb); }






/**
 * @brief ������Ʋ��Ժ���
 */
void motorTest(void)
{
    motorInit();            // ��ʼ�����ϵͳ
//    leftMotorControl(500);  // ����50%ռ�ձ�
//    rightMotorControl(500); // �ҵ��50%ռ�ձ�
}


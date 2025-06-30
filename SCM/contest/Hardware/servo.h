#ifndef __SERVO_H
#define __SERVO_H


#include "stm32f1xx_hal.h"

// Ĭ�϶������
#define SERVO_MIN_PULSE      50     // 0.5ms
#define SERVO_MAX_PULSE      250    // 2.5ms

/* ������ýṹ�� */
typedef struct {
    TIM_HandleTypeDef *htim;     // ��ʱ�����
    uint32_t channel;            // PWMͨ��
    uint16_t min_pulse;          // ��С������ (��s)
    uint16_t max_pulse;          // ��������� (��s)
    float min_angle;             // ��С�Ƕ� (��)
    float max_angle;             // ���Ƕ� (��)
} Servo_Config;

/* �������� */
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void Servo_SetPulse(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us);
void Servo_Calibrate(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t min_pulse, uint16_t max_pulse);

#endif /* __SERVO_H */

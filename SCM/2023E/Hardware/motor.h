#ifndef _MOTOR_H
#define _MOTOR_H
 
#include "stm32f1xx_hal.h"
#include "global.h"
#include "myMath.h"

void motorTest(void);
void leftMotorControl(int compare_value);
void rightMotorControl(int compare_value);
float leftWheelSpeed(void);
float rightWheelSpeed(void);
void motorInit(void);
float getWheelSpeed(TIM_HandleTypeDef *htim, EncoderState *state);



// 全局编码器状态（左右电机各一个）
extern EncoderState left_encoder;
extern EncoderState right_encoder;


#endif



#ifndef _MOTOR_H
#define _MOTOR_H
#include "global.h"
#include "myMath.h"
void Step_Init(void);
void Set_Direction(StepperMotor* motor, uint8_t dir);
void Set_Speed(StepperMotor* motor, uint32_t freq_hz);
void Move_Angle(StepperMotor* motor, float angle, uint32_t freq);
void Move_Angle_Global(StepperMotor* motor,float angle, uint32_t freq);
void move_one_step(StepperMotor* motor,uint8_t dir,uint32_t freq);
void move_steps(StepperMotor* motor,int32_t steps,uint32_t freq);
void Reset_Position(void);
void generate_linear_path(Point2D_t start, Point2D_t end, Angles_t* path);
void Reset_Gimbal(void);
void PID_motors(int16_t offset_x,int16_t offset_y);
void PID_motors2(int16_t offset_x,int16_t offset_y);
HAL_StatusTypeDef Motor_Enable_X(uint8_t address, uint8_t enable, uint8_t sync_flag);
HAL_StatusTypeDef Speed_Mode_X(uint8_t address, uint8_t direction, uint16_t speed, uint8_t acceleration, uint8_t sync_flag);
HAL_StatusTypeDef Position_Mode_X(uint8_t address, uint8_t direction, uint16_t speed, uint8_t acceleration, uint32_t pulses, uint8_t abs_mode, uint8_t sync_flag);
HAL_StatusTypeDef Motor_Stop_X(uint8_t address, uint8_t sync_flag);
HAL_StatusTypeDef Sync_Move_X(uint8_t address);
HAL_StatusTypeDef Set_Home_Position_X(uint8_t address, uint8_t store_flag);
HAL_StatusTypeDef Trigger_Home_X(uint8_t address, uint8_t home_mode, uint8_t sync_flag);
HAL_StatusTypeDef Abort_Home_X(uint8_t address);
HAL_StatusTypeDef Send_Get_Motor_Speed_X(uint8_t address);
HAL_StatusTypeDef Send_Get_Motor_Position_X(uint8_t address);
HAL_StatusTypeDef Motor_Enable_Y(uint8_t address, uint8_t enable, uint8_t sync_flag);
HAL_StatusTypeDef Speed_Mode_Y(uint8_t address, uint8_t direction, uint16_t speed, uint8_t acceleration, uint8_t sync_flag);
HAL_StatusTypeDef Position_Mode_Y(uint8_t address, uint8_t direction, uint16_t speed, uint8_t acceleration, uint32_t pulses, uint8_t abs_mode, uint8_t sync_flag);
HAL_StatusTypeDef Motor_Stop_Y(uint8_t address, uint8_t sync_flag);
HAL_StatusTypeDef Sync_Move_Y(uint8_t address);
HAL_StatusTypeDef Set_Home_Position_Y(uint8_t address, uint8_t store_flag);
HAL_StatusTypeDef Trigger_Home_Y(uint8_t address, uint8_t home_mode, uint8_t sync_flag);
HAL_StatusTypeDef Abort_Home_Y(uint8_t address);
HAL_StatusTypeDef Send_Get_Motor_Speed_Y(uint8_t address);
HAL_StatusTypeDef Send_Get_Motor_Position_Y(uint8_t address);
void Get_Motor_Speed(uint8_t address,uint8_t* data);
void Get_Motor_Position(uint8_t address,uint8_t* data);
/* 角度控制函数（相对角度） */
HAL_StatusTypeDef Move_Relative_Angle_X(uint8_t address, float angle, uint16_t speed);
/* 角度控制函数（绝对角度） */
HAL_StatusTypeDef Move_Absolute_Angle_X(uint8_t address, float angle, uint16_t speed);
/* 角度控制函数（相对角度） */
HAL_StatusTypeDef Move_Relative_Angle_Y(uint8_t address, float angle, uint16_t speed);
/* 角度控制函数（绝对角度） */
HAL_StatusTypeDef Move_Absolute_Angle_Y(uint8_t address, float angle, uint16_t speed);

void my_move_steps(StepperMotor* motor,uint32_t steps,uint8_t dir,uint32_t freq);


int arrive_target(uint16_t x_t,uint16_t y_t,uint16_t x_c,uint16_t y_c,int dis);
extern StepperMotor motor_x; 
extern StepperMotor motor_y;
extern MotorData motor_data[2];
#endif



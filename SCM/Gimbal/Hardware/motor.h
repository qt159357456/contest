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
void Reset_Position(void) ;
void generate_linear_path(Point2D_t start, Point2D_t end, Angles_t* path);
void Reset_Gimbal(void);
void PID_motors(uint16_t x_t,uint16_t y_t,uint16_t x_c,uint16_t y_c);
int arrive_target(uint16_t x_t,uint16_t y_t,uint16_t x_c,uint16_t y_c,int dis);
extern StepperMotor motor_x; 
extern StepperMotor motor_y;
#endif



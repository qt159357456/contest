#ifndef __SERVO_H
#define __SERVO_H
#include "global.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

void Servo_Init(void);
void Move_To_Point(Point target);
void Reset_Position(void);
void Move_Square_Path(void);
void Move_Custom_Path(const Point* path, uint16_t points);


#endif

#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "i2c.h"

uint8_t MPU6050_Init(void); 
void MPU6050_Read_Accel(float *Accel); 
void MPU6050_Read_Gyro(float *Gyro);

#endif


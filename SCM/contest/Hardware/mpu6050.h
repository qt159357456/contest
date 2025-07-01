#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "i2c.h"

//uint8_t MPU6050_Init(void); 
void MPU6050_Read_Accel(float *Accel); 
void MPU6050_Read_Gyro(float *Gyro);
void MPU6050_Init(void);
void MPU6050_GetData(uint8_t *Arr);
void convert_uint8_to_uint16(float *Accel,float *Gyro);
#endif


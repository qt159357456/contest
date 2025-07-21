#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "global.h"
#include "i2c.h"


#define	MPU6050_SMPLRT_DIV 0x19
#define	MPU6050_CONFIG 0x1A
#define	MPU6050_GYRO_CONFIG 0X1B
#define	MPU6050_ACCEL_CONFIG 0x1C
#define	MPU6050_ACCEL_XOUT_H 0X3B
#define	MPU6050_ACCEL_XOUT_L 0x3C
#define	MPU6050_ACCEL_YOUT_H 0X3D
#define	MPU6050_ACCEL_YOUT_L 0X3E
#define	MPU6050_ACCEL_ZOUT_H 0X3F
#define	MPU6050_ACCEL_ZOUT_L 0x40
#define	MPU6050_TEMP_OUT_H 0x41
#define	MPU6050_TEMP_OUT_L 0x42
#define	MPU6050_GYRO_XOUT_H 0x43
#define	MPU6050_GYRO_XOUT_L 0x44
#define	MPU6050_GYRO_YOUT_H 0X45
#define	MPU6050_GYRO_YOUT_L 0x46
#define	MPU6050_GYRO_ZOUT_H 0x47
#define	MPU6050_GYRO_ZOUT_L 0x48
#define	MPU6050_PWR_MGMT_1 0x6B
#define	MPU6050_PWR_MGMT_2 0x6C
#define	MPU6050_WHO_AM_I 0x75

#define MPU6050_ADDR 0x68 // 使用7位设备地址（0x68或0x69）
#define RtA 57.2957795f  // 弧度转角度
#define AtR 0.0174532925f  // 角度转弧度


void MPU6050_Read_Accel(INT16_XYZ_t *value); 
void MPU6050_Read_Gyro(INT16_XYZ_t *value);
void MPU6050_Init(void);
void MPU6050_GetData(uint8_t *Arr);
void CalibrateMPU6050(void);


void MPU6050_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az); 
void MPU6050_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);
float MPU6050_ConvertAccel(float value, uint8_t range);
float MPU6050_ConvertGyro(float value, uint8_t range);
void MPU6050_ReadSensors(MPU6050_Data_t *Mpu);
void NormalizeAccel(float *ax, float *ay, float *az);
void ComputeError(float ax, float ay, float az, float *error_x, float *error_y, float *error_z);
void UpdateQuaternion(float gx, float gy, float gz, float error_x, float error_y, float error_z, float dt);
void ComputeEulerAngles(float *pitch, float *roll, float *yaw);
void GetAngles(float *pitch, float *roll, float *yaw, float dt);
void MPU6050_Read_Accel_Raw(int16_t *Accel);
void MPU6050_Read_Gyro_Raw(int16_t *Gyro);



void GetAngle(const MPU6050_Data_t *pMpu,Angle_t *pAngE, float dt);
void imu_rest(void);


void MPU6050_Offset(void);
void MPU6050_GyroRead(FLOAT_XYZ_t *Gyr_filt);
void MPU6050_AccRead(FLOAT_XYZ_t *Acc_filt);

extern INT16_XYZ_t MPU6050_ACC_RAW;
extern INT16_XYZ_t MPU6050_GYRO_RAW;
#endif


/************************************************************************************************
* 程序版本：V3.0
* 程序日期：2022-11-3
* 程序作者：719飞行器实验室
************************************************************************************************/
#ifndef   _IMU_H_
#define   _IMU_H_
#include "main.h"
#include "global.h"
#include "mpu6050.h"
#include <math.h>
#include "myMath.h"


#define G			9.80665f		      	// m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)

void Prepare_Data(void);
void IMUupdate(FLOAT_XYZ_t *Gyr_filt,FLOAT_XYZ_t *Acc_filt,Angle_t *Att_Angle,float dt);


extern Angle_t Att_Angle;  
extern FLOAT_XYZ_t Acc_filt;
extern FLOAT_XYZ_t	Gyr_filt;
#endif

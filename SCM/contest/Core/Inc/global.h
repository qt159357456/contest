#ifndef 	GLOBAL_H
#define   GLOBAL_H
#include "stdint.h"


typedef uint8_t u8;




typedef struct {
	float Target;
	float Actual;
	float Actual1;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt;
	
	float ErrorIntMax;
	float ErrorIntMin;
	float OutMax;
	float OutMin;
	
} PID_t;

typedef struct {
	float Alpha;
	double Pre_out;
	double Pre_in;
}Filter_t;

//滑动均值滤波
#define AVG_LEN 10
typedef struct {
	double Data[AVG_LEN];
	uint8_t index;
	double sum;
}AVG_Flt_t;

//中值滤波
#define MEDIAN_LEN 10
typedef struct {
	double Data[MEDIAN_LEN];
	uint8_t index;
}MEDIAN_Flt_t;

////带通滤波
//#define N 101
//#define PI 3.1415926535897932384626
//typedef struct {
//	float Fs;
//	float Fc1;
//	float Fc2;
//	float b[N];
//	float buffer[N];
//	int index;
//}FIR_Flt_t;

//卡尔曼滤波
#define PI 3.1415926535897932384626
typedef struct{
	float P;
	float G;
	float Q;
	float R;
	float Output;
}Klm_Flt_t;

#endif


























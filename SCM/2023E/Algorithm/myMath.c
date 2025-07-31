#include "myMath.h"




/****************************************************************************************************
* 函  数：float invSqrt(float x) 
* 功　能: 快速计算 1/Sqrt(x) 	
* 参  数：要计算的值
* 返回值：计算的结果
* 备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*****************************************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


// 限幅函数
float constrain(float val, float min, float max) {
    return val < min ? min : (val > max ? max : val);
}


void distance_to_angle(float x, float y,float* pitch_angle, float* yaw_angle) {
    const float real_length = 1000.0; //单位毫米
    
    // 计算像素偏移（图像坐标系，Y轴向下）
    double dx = -x;   // X轴偏移：向左为正角
    double dy = y;   // Y轴偏移：向上为正角

    // 转换为物理角度（弧度）
    double yaw_rad = atan2(dx, real_length);    // 偏航角
    double pitch_rad = atan2(dy, real_length); // 俯仰角
    
    // 转换为角度（度）
    *yaw_angle = yaw_rad * (180.0 / PI);
    *pitch_angle = pitch_rad * (180.0 / PI);
}

void distance_to_angle2(Point2D_t point,Angles_t* angle) {
    const float real_length = 1000.0; //单位毫米
    
    // 计算像素偏移（图像坐标系，Y轴向下）
    double dx = -point.x;   // X轴偏移：向左为正角
    double dy = point.y;   // Y轴偏移：向上为正角

    // 转换为物理角度（弧度）
    double yaw_rad = atan2(dx, real_length);    // 偏航角
    double pitch_rad = atan2(dy, real_length); // 俯仰角
    
    // 转换为角度（度）
    angle->yaw = yaw_rad * (180.0 / PI);
    angle->pitch = pitch_rad * (180.0 / PI);
}

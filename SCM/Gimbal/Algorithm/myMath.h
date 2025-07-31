#ifndef __MY_MATH_H
#define	__MY_MATH_H
#include <math.h>
#include "global.h"
float constrain(float val, float min, float max); 


float invSqrt(float x);
/**
 * @brief ·ûºÅº¯Êı
 * @param x ÊäÈëÖµ
 * @return -1 (x < 0), 0 (x == 0), 1 (x > 0)
 */
static inline float Sign_Judge(float x)
{
    if(x > 0.0f) return 1.0f;
    if(x < 0.0f) return -1.0f;
    return 0.0f;
}
void distance_to_angle(float x, float y,float* pitch_angle, float* yaw_angle);
void distance_to_angle2(Point2D_t point,Angles_t* angle) ;
#endif /* __Algorithm_math_H */

#ifndef __MY_MATH_H
#define	__MY_MATH_H

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
#endif /* __Algorithm_math_H */

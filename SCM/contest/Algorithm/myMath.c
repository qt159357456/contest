#include "myMath.h"
#include <math.h>




/****************************************************************************************************
* ��  ����float invSqrt(float x) 
* ������: ���ټ��� 1/Sqrt(x) 	
* ��  ����Ҫ�����ֵ
* ����ֵ������Ľ��
* ��  ע������ͨSqrt()����Ҫ���ı�See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
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


// �޷�����
float constrain(float val, float min, float max) {
    return val < min ? min : (val > max ? max : val);
}

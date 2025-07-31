#include "myMath.h"




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


void distance_to_angle(float x, float y,float* pitch_angle, float* yaw_angle) {
    const float real_length = 1000.0; //��λ����
    
    // ��������ƫ�ƣ�ͼ������ϵ��Y�����£�
    double dx = -x;   // X��ƫ�ƣ�����Ϊ����
    double dy = y;   // Y��ƫ�ƣ�����Ϊ����

    // ת��Ϊ����Ƕȣ����ȣ�
    double yaw_rad = atan2(dx, real_length);    // ƫ����
    double pitch_rad = atan2(dy, real_length); // ������
    
    // ת��Ϊ�Ƕȣ��ȣ�
    *yaw_angle = yaw_rad * (180.0 / PI);
    *pitch_angle = pitch_rad * (180.0 / PI);
}

void distance_to_angle2(Point2D_t point,Angles_t* angle) {
    const float real_length = 1000.0; //��λ����
    
    // ��������ƫ�ƣ�ͼ������ϵ��Y�����£�
    double dx = -point.x;   // X��ƫ�ƣ�����Ϊ����
    double dy = point.y;   // Y��ƫ�ƣ�����Ϊ����

    // ת��Ϊ����Ƕȣ����ȣ�
    double yaw_rad = atan2(dx, real_length);    // ƫ����
    double pitch_rad = atan2(dy, real_length); // ������
    
    // ת��Ϊ�Ƕȣ��ȣ�
    angle->yaw = yaw_rad * (180.0 / PI);
    angle->pitch = pitch_rad * (180.0 / PI);
}

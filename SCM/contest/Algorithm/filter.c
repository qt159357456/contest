#include "filter.h"
#include "math.h"

//��ͨ�˲�
double Low_Pass_Filter(Filter_t * filter,double input)
{
	double output = filter->Alpha * filter->Pre_out + (1-filter->Alpha)*input;
	filter->Pre_out = output;
	return output;
}

//��ͨ�˲�
double High_Pass_Filter(Filter_t * filter,double input)
{
	double output = filter->Alpha * (input - filter->Pre_in) + filter->Alpha*filter->Pre_out;
	filter->Pre_in = input;
	filter->Pre_out = output;
	return output;
}

//������ֵ�˲�
float vectorFilter(float new_speed, AVG_Flt_t *rb)
{
    // �����ܺͣ��Ƴ���ֵ��������ֵ
    *(rb->sum) -= rb->buffer[*(rb->index)]; // ��ȥ��������
    rb->buffer[*(rb->index)] = new_speed;   // д��������
    *(rb->sum) += new_speed;                // ����������

    // ���»���������ѭ�����У�
    *(rb->index) = (*(rb->index) + 1) % rb->size;

    // ���ص�ǰƽ��ֵ
    return *(rb->sum) / rb->size;
}

//��ֵ�˲�
//�ȽϺ���������qsort����
int compare(const void *a,const void *b)
{
	double da = *(const double *)a;
	double db = *(const double *)b;
	return(da>db)-(da<db);
}

double Median_Filter(MEDIAN_Flt_t * filter,double input)
{
	uint8_t i = 0;
	double median = 0;
	
	//�����������ݴ�������
	filter->Data[filter->index++] = input;
	
	//��������������鷶Χ����ѭ�������鿪ͷ
	if(filter->index >= MEDIAN_LEN)
	{
		filter->index = 0;
	}
	
	//����һ����ʱ������������
	double tempData[MEDIAN_LEN];
	for(i = 0;i < MEDIAN_LEN;i++)
	{
		tempData[i] = filter->Data[i];
	}
	
	//����ʱ�����������
	qsort(tempData,MEDIAN_LEN,sizeof(double),compare);
	
	//ȡ��ֵ
	median = tempData[MEDIAN_LEN / 2];
	
	return median;
}

//�������˲�
double Kalman_Filter(Klm_Flt_t * EKF,double input)
{
	EKF->NewP = EKF->LastP + EKF->Q;
	EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
	EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
	EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
	
	return EKF->Out;
}





































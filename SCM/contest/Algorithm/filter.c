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
double Average_Filter(AVG_Flt_t * filter,double input)
{
	double avg = 0;
	
	filter->sum -= filter->Data[filter->index];
	filter->Data[filter->index++] = input;
	filter->sum += input;
	if(filter->index >= AVG_LEN){
		filter->index = 0;
	}

	avg = filter->sum / AVG_LEN;
	
	return avg;
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
double Kalman_Filter(Klm_Flt_t * kfp,double input)
{
	//����Э����̣���ǰ����Э����=�ϴθ���Э����+��������Э����
	kfp->P = kfp->P + kfp->Q;
	
	//���������淽�̣���ǰ����������=��ǰ����Э����/����ǰ����Э����+��������Э���
	kfp->G = kfp->P / (kfp->P + kfp->R);
	
	//��������ֵ���̣���ǰ����ֵ=��ǰ����ֵ+����������*����ǰ����ֵ-��ǰ����ֵ��
	kfp->Output = kfp->Output + kfp->G * (input - kfp->Output);//��ǰ����ֵ=�ϴ�����ֵ
	
	//����Э����=��1-���������棩*��ǰ����Э����
	kfp->P = (1 - kfp->G) * kfp->P;
	
	return kfp->Output;
}





































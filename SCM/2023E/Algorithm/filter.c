#include "filter.h"
#include "math.h"

//低通滤波
double Low_Pass_Filter(Filter_t * filter,double input)
{
	double output = filter->Alpha * filter->Pre_out + (1-filter->Alpha)*input;
	filter->Pre_out = output;
	return output;
}

//高通滤波
double High_Pass_Filter(Filter_t * filter,double input)
{
	double output = filter->Alpha * (input - filter->Pre_in) + filter->Alpha*filter->Pre_out;
	filter->Pre_in = input;
	filter->Pre_out = output;
	return output;
}

//滑动均值滤波
float vectorFilter(float new_speed, AVG_Flt_t *rb)
{
    // 更新总和：移除旧值，加入新值
    *(rb->sum) -= rb->buffer[*(rb->index)]; // 减去过期数据
    rb->buffer[*(rb->index)] = new_speed;   // 写入新数据
    *(rb->sum) += new_speed;                // 加上新数据

    // 更新环形索引（循环队列）
    *(rb->index) = (*(rb->index) + 1) % rb->size;

    // 返回当前平均值
    return *(rb->sum) / rb->size;
}

//中值滤波
//比较函数，用于qsort排序
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
	
	//将新输入数据存入数组
	filter->Data[filter->index++] = input;
	
	//如果索引超出数组范围，则循环回数组开头
	if(filter->index >= MEDIAN_LEN)
	{
		filter->index = 0;
	}
	
	//创建一个临时数组用于排序
	double tempData[MEDIAN_LEN];
	for(i = 0;i < MEDIAN_LEN;i++)
	{
		tempData[i] = filter->Data[i];
	}
	
	//对临时数组进行排序
	qsort(tempData,MEDIAN_LEN,sizeof(double),compare);
	
	//取中值
	median = tempData[MEDIAN_LEN / 2];
	
	return median;
}

//卡尔曼滤波
double Kalman_Filter(Klm_Flt_t * EKF,double input)
{
	EKF->NewP = EKF->LastP + EKF->Q;
	EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
	EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
	EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
	
	return EKF->Out;
}





































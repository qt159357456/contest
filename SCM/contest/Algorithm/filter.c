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
double Kalman_Filter(Klm_Flt_t * kfp,double input)
{
	//估算协方差方程：当前估算协方差=上次更新协方差+过程噪声协方差
	kfp->P = kfp->P + kfp->Q;
	
	//卡尔曼增益方程：当前卡尔曼增益=当前估算协方差/（当前估算协方差+测量噪声协方差）
	kfp->G = kfp->P / (kfp->P + kfp->R);
	
	//更新最优值方程：当前最优值=当前估算值+卡尔曼增益*（当前测量值-当前估算值）
	kfp->Output = kfp->Output + kfp->G * (input - kfp->Output);//当前估算值=上次最优值
	
	//更新协方差=（1-卡尔曼增益）*当前估算协方差
	kfp->P = (1 - kfp->G) * kfp->P;
	
	return kfp->Output;
}





































#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "math.h"

void PID_Update(PID_t *p)
{
	float C;
	float k=1;
	float DifOut;
	p->Actual1 = p->Actual;
	//获取本次误差和上次误差
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	//变速积分
	C = 1 / ( k * fabs(p->Error0) + 1);
	
	//误差积分
		p->ErrorInt += p->Error0;

	//微分先行
	DifOut = - p->Kd * ( p->Actual - p->Actual1 );
	
	//积分限幅
	if (p->ErrorInt > p->ErrorIntMax) {p->ErrorInt = p->ErrorIntMax;}
	if (p->ErrorInt < p->ErrorIntMin) {p->ErrorInt = p->ErrorIntMin;}
	
	//PID计算
	p->Out = p->Kp * p->Error0
		   + C * p->Ki * p->ErrorInt
		   + DifOut;
	
	//输出限幅
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}


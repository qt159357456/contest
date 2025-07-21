#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "math.h"

void PID_Update(PID_t *p)
{
    // 获取本次误差和上次误差
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    
    // 积分累计
		if(fabs(p->Error0) > p->IntMinThreshold)
				p->ErrorInt += p->Error0;
    
    // 积分限幅
    if (p->ErrorInt > p->ErrorIntMax) p->ErrorInt = p->ErrorIntMax;
    if (p->ErrorInt < p->ErrorIntMin) p->ErrorInt = p->ErrorIntMin;
		
    
    // PID计算
    p->pOut = p->Kp * p->Error0;
    p->iOut = p->Ki * p->ErrorInt;  
		p->dOut = p->Kd * (p->Error0 - p->Error1);
    p->Out = p->pOut + p->iOut + p->dOut;
    
    // 输出限幅
    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;
		    // 保存上次实际值并更新当前实际值
    p->Actual1 = p->Actual;
}

void Clip_TD_Function(TD *pstTd, float lim_x2)
{
	float d, d0, y, a0, a = 0;
	pstTd->x = pstTd->x1 - pstTd->aim;//当前值减目标值
	d = pstTd->r * pstTd->h;//加速度乘以滤波周期
	d0 = pstTd->h * d;//
	y = pstTd->x + pstTd->h * pstTd->x2;
	a0 = sqrt(d * d + 8 * pstTd->r * fabs(y));

	if (fabs(y) > d0)
		a = pstTd->x2 + (a0 - d) * Sign_Judge(y) / 2;
	else
		a = pstTd->x2 + y / pstTd->h;

	if (fabs(a) > d)
		y = -1 * pstTd->r * Sign_Judge(a);
	else
		y = -1 * pstTd->r * a / d;
	lim_x2 = fabs(lim_x2);
	if (pstTd->x2 > lim_x2)
	{
		pstTd->x2 = lim_x2;
	}
	else if (pstTd->x2 < -lim_x2)
	{
		pstTd->x2 = -lim_x2;
	}
	pstTd->x1 += pstTd->T * pstTd->x2;	
	pstTd->x2 += pstTd->T * y;
	pstTd->x3 = y;
	
	if(pstTd->aim>0){
			if(pstTd->x1>pstTd->aim){
					pstTd->x1 = pstTd->aim;
					pstTd->x2 = 0;
			}
	}
	else if(pstTd->aim<0){
			if(pstTd->x1<pstTd->aim){
					pstTd->x1 = pstTd->aim;
					pstTd->x2 = 0;
			}
	}
	
}

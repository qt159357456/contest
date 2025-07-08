#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "math.h"

PID_t angle_pid;

void PID_Update(PID_t *p)
{
    float deltaTarget;
    static float lastTarget = 0;  // 静态变量保存上次目标值
    float DifOut;
	
		p->TargetChangeThreshold = 0.5;

    // 检测目标值突变（首次运行需要初始化 lastTarget）
    if(lastTarget != 0) {  // 确保不是第一次运行
        deltaTarget = fabs(p->Target - lastTarget);
        if(deltaTarget > p->TargetChangeThreshold) {
            p->ErrorInt = 0;  // 目标变化过大，清零积分累计
        }
    }
    lastTarget = p->Target;  // 更新记录的目标值
    // 获取本次误差和上次误差
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    
    // 积分累计（取消变速积分）
    p->ErrorInt += p->Error0;
    
    // 微分先行
    DifOut = -p->Kd * (p->Actual - p->Actual1);
    
    // 积分限幅
    if (p->ErrorInt > p->ErrorIntMax) p->ErrorInt = p->ErrorIntMax;
    if (p->ErrorInt < p->ErrorIntMin) p->ErrorInt = p->ErrorIntMin;
    
    // PID计算
    p->pOut = p->Kp * p->Error0;
    p->iOut = p->Ki * p->ErrorInt;  // 取消变速积分
    p->dOut = DifOut;
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

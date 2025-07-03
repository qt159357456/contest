#include "stm32f10x.h"                  // Device header
#include "PID.h"
#include "math.h"

PID_t angle_pid;

void PID_Update(PID_t *p)
{
	float C;
	float k=1;
	float DifOut;
	p->Actual1 = p->Actual;
	//��ȡ���������ϴ����
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	//���ٻ���
	C = 1 / ( k * fabs(p->Error0) + 1);
	
	//������
		p->ErrorInt += p->Error0;

	//΢������
	DifOut = - p->Kd * ( p->Actual - p->Actual1 );
	
	//�����޷�
	if (p->ErrorInt > p->ErrorIntMax) {p->ErrorInt = p->ErrorIntMax;}
	if (p->ErrorInt < p->ErrorIntMin) {p->ErrorInt = p->ErrorIntMin;}
	
	//PID����
	p->Out = p->Kp * p->Error0
		   + C * p->Ki * p->ErrorInt
		   + DifOut;
	
	//����޷�
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}

void Clip_TD_Function(TD *pstTd, float lim_x2)
{
	float d, d0, y, a0, a = 0;
	pstTd->x = pstTd->x1 - pstTd->aim;//��ǰֵ��Ŀ��ֵ
	d = pstTd->r * pstTd->h;//���ٶȳ����˲�����
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

#ifndef __PID_H
#define __PID_H
#include "global.h"
#include "myMath.h"

void PID_Update(PID_t *p);
extern PID_t angle_pid;
void Clip_TD_Function(TD *pstTd, float lim_x2);

#endif


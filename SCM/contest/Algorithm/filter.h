#ifndef __FILTER_H
#define __FILTER_H
#include "global.h"
#include "main.h"
#include "stdlib.h"

double Low_Pass_Filter(Filter_t * filter,double input);
double High_Pass_Filter(Filter_t * filter,double input);
double Average_Filter(AVG_Flt_t * filter,double input);
double Median_Filter(MEDIAN_Flt_t * filter,double input);
//void FIRBandPassFilter_Init(FIR_Flt_t * filter);
//double FIR_BandPass_Filter(FIR_Flt_t * filter,double input);
double Kalman_Filter(Klm_Flt_t * kfp,double input);
#endif

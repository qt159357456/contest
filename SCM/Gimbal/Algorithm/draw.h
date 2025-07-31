#ifndef __DRAW_H
#define __DRAW_H
#include "stdint.h"

void DrawLine(int x0,int y0,int x1,int y1,uint8_t draw);
void DrawSin(void);
void DrawCircle(int x0,int y0,int R);
void DrawRoundRectangle(int x0,int y0,int x1,int y1,int R);
#endif

#include "draw.h"
#include "math.h"
#include "stdlib.h"


void DrawSin(){
	int x0,y0;
	float Si,Sx,Sy,ax = 1.0,ay = 1.0;
	for(Si = -180;Si < 180;Si++){
		Sx = Si * ax;
		Sy = sin(Si * 3.1416f / 180) * ay;
//		DrawPoint(Sx + x0,Sy + y0);
//		Refresh();
	}
}

void DrawLine(int x0,int y0,int x1,int y1,uint8_t draw){
	int lengthx,lengthy,dm,i;
	float dx,dy,x,y;
	lengthx = abs(x1 - x0);
	lengthy = abs(y1 - y0);
	
	if(lengthx >= lengthy){
		dm = lengthx;
	}
	else{
		dm = lengthy;
	}
		
	dx = (float)(x1 - x0) / dm;
	dy = (float)(y1 - y0) / dm;	
	x = (float)x0 + 0.5f;
	y = (float)y0 + 0.5f;
	
	for(i = 0;i <= dm;i++){
//		if(draw)
//		DrawPoint(x,y);
//		else
//    ClearPoint(x,y);
			x += dx;
			y += dy;
	}
}

void DrawCircle(int x0,int y0,int R){
	int x,y,d;	
	x = 0;
	y = R;
	d = 3 - 2 * R;
	while(x <= y){
//		DrawPoint(x + x0,y + y0);
//		DrawPoint(-x + x0,y + y0);
//		DrawPoint(-x + x0,-y + y0);
//		DrawPoint(x + x0,-y + y0);
//		DrawPoint(y + x0,x + y0);
//		DrawPoint(-y + x0,x + y0);
//		DrawPoint(-y + x0,-x + y0);
//		DrawPoint(y + x0,-x + y0);
		if(d < 0){
			d += 4 * x + 6;
		}
		else{
			d += 4 * (x - y) + 10;
			y--;
		}
		x++;
	}
//		Refresh();	
}

void DrawRoundRectangle(int x0,int y0,int x1,int y1,int R){
	int x,y,d;	
	int xstart,ystart,xend,yend;
	
	if(x0 < x1){
		xstart = x0;
		xend = x1;
	}
	else{
		xstart = x1;
		xend = x0;
	}
	if(y0 < y1){
		ystart = y0;
		yend = y1;
	}
	else{
		ystart = y1;
		yend = y0;
	}
	
	if(R > (xend - xstart) / 2 || R > (yend - ystart) / 2){
		R = ((xend - xstart) < (yend - ystart)) ? ((xend - xstart) / 2) : ((yend - ystart) / 2);
	}
		
	x = 0;
	y = R;
	d = 3 - 2 * R;
	while(x <= y){
//		DrawPoint(xstart + R - x,ystart + R - y);
//		DrawPoint(xstart + R - y,ystart + R - x);
//		DrawPoint(xend - R + x,ystart + R - y);
//		DrawPoint(xend - R + y,ystart + R - x);
//		DrawPoint(xstart + R - x,yend - R + y);
//		DrawPoint(xstart + R - y,yend - R + x);
//		DrawPoint(xend - R + x,yend - R + y);
//		DrawPoint(xend - R + y,yend - R + x);
		if(d < 0){
			d += 4 * x + 6;
		}
		else{
			d += 4 * (x - y) + 10;
			y--;
		}
		x++;
	}
		DrawLine(xstart + R + 1,ystart,xend - R - 1,ystart,1);
		DrawLine(xstart + R + 1,yend,xend - R - 1,yend,1);
		DrawLine(xstart,ystart + R + 1,xstart,yend - R - 1,1);
	  DrawLine(xend,ystart + R + 1,xend,yend - R - 1,1);
	
//		Refresh();			
}


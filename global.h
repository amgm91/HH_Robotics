#ifndef _GLOBAL_H
#define _GLOBAL_H

#include <math.h>

typedef struct {
	float X;
	float Y;
	float Theta;
} TYPE_POS;




enum TYPE_STATE {
	Start,
	Find_Obj,
	Move_2_Object,
};



extern enum TYPE_STATE Robot_State;

double Local_Time(void);

#endif

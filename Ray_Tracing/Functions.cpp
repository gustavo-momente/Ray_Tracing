#include "Functions.h"

//Used to see if two numbers are equal if they are in a neighborhood defined by DIX_FLT_EPS

bool EqCompare(float x,float y)
{
#ifdef DIX_FLT_EPS
	if(x == y || (x < y+DIX_FLT_EPS && x > y-DIX_FLT_EPS))
#else
	if(x == y)
#endif
		return true; 
	return false;
}

//Same thing as EqCompare, but with a different define
//Used to detect collision with the light

bool CompareLight(float x,float y)
{
#ifdef LIGHT_EPS
	if(x == y || (x < y+LIGHT_EPS && x > y-LIGHT_EPS))
#else
	if(x == y)
#endif
		return true; 
	return false;
}

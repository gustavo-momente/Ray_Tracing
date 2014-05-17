
#ifndef DEFINES_H_
#define DEFINES_H_

#include <algorithm>

//The next six flags are just to let the program more readable
#define PARALLEL_LINE 1
#define LINE_IN_PLANE 2
#define ONE_POINT_INSTERSECTION 0
#define NOT_PERMENDICULAR 14
#define REFRACTIVE 1
#define NOT_REFRACTIVE 0

//When an IRay is reflected/refracted it can't be returned in the same exact point where 
//it hit, because numerical error enters in the game and makes the program think that the next
// collision happens in the same spot. So, we workaround this by displacing the outgoing ray origin
//by Delta times his direction
#define DELTA float(0.001)  

//The next two flags add an confidence interval to consider if a collision occurs with the light (LIGHT_EPS)
//or if a point belongs to a object or if a collision occurs (DIX_FLT_EPS)
#define DIX_FLT_EPS 10*FLT_EPSILON
#define LIGHT_EPS 0.001

//Number of interaction a ray can make before perish
#define REFLEX_COUNTER 10

//DEBUG is used to include Visual Leak Detector (http://vld.codeplex.com/)
//an useful tool to detect memory leaks
//#define DEBUG
#ifdef DEBUG
	#include <vld.h> 
#endif  

//PARALLEL enables multi-processor using
//#define PARALLEL

//If TWOWAY is defined, when there is a diffuse reflection instead of returning a single IRay
//the reflection will return two. Both having half the normal intensity, but one of them will
//consider refractive objects as transparent and don't refract.
//This was implemented, because is almost impossible to figure out what outgoing direction a ray
//must take to pass through an refractive object and hit a light source.
//So we try to give a more "natural" feel by sending the two rays straight to the light, as if
//there isn't a refractive object in the way, with one always hitting the light and the other 
//following the comportment imposed by the refraction

//#define TWOWAY 
#ifdef TWOWAY
	#define TWOWAY_MUL 0.5f
#else
	#define TWOWAY_MUL 1
#endif //TWOWAY

//N1 defines the refractive index of the medium where the scene is
#define N1 1

//if SHOW_OUTPUT is defined a windows will appear after the rendering with the scene's outcome
//It's not really useful if the user want to run some scripts with the program 
#define SHOW_OUTPUT
#endif // !DEFINES_H_

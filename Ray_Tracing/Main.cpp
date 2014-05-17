#include "opencv2/opencv.hpp"
#include <vector>
#include <stack>
#include <stdlib.h> 
#include <fstream>
#include "primitives.h"
#include "Ray_tracer.h" 
#include "openCV_test.h"


using namespace std;


int main(int argc, char* argv[])
{
	if (argc < 3 ) 
	{ 
		printf(" Error : User must enter an input and an output file!! \n Incorrect number of arguments " );
		return -1;
	}

	Scene* scene = new Scene();
	scene->ReadScene ( argv[1] );
	Ray_tracer tracer(scene);
	if (argc == 3 )
	tracer.SetOutput(argv[2]);
	tracer.Tracer();

	return 0;
}

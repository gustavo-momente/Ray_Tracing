//#include "stdafx.h"
#ifndef openCV_test
#define openCV_test
#include "openCV_test.h"
#endif // !openCV_test

// Function to test if openCV is working as indeed
// it should open a windows and display your camera
// Remember to change the compile target to "Release" 
void openCV_working_test()
{
		VideoCapture cap(0); // open the default camera
	if(!cap.isOpened()){ // check if we succeeded
		printf("Couldn't open the default camera.\n");}
	Mat frame;
	namedWindow("Original",  CV_WINDOW_AUTOSIZE);
	while(true)
	{
		cap >> frame; 	
		imshow("Original", frame);

		if(waitKey(30) >= 0) break;
	}

}
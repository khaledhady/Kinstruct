#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "Transformation.h"
#include <math.h>

#ifndef HORN
#define HORN

using namespace std;

class HornMethod
{

	public:
		HornMethod();
		void getTransformation(vector<cv::Point3f> *setA, vector<cv::Point3f> *setB, Transformation *Result );
		void selfTest();

	private:
};

#endif
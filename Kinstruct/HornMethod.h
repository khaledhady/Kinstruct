#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Visualizer.h"
#include "Transformation.h"
#include <math.h>
using namespace cv;
using namespace std;

class HornMethod
{

	public:
		HornMethod();
		void getTransformation(vector<Point3f*> setA, vector<Point3f*> setB, Transformation *Result );
		void selfTest();

	private:
};
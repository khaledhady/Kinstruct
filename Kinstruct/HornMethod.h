#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
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
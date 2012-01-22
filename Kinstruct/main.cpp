#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Visualizer.h"
#include "RobustMatcher.h"
#include "HornMethod.h"
#define START_FRAME 100
#define STEP 50
#define LAST_FRAME 199
using namespace cv;
using namespace std;

/* Using the GLUT library for the base windowing setup */
#include <GL/glut.h>
#include <GL/gl.h>
#include <math.h>
void alignFrames(Mat *colorA, Mat *colorB, Mat *depthA, Mat *depthB, Transformation *inverse)
{
	// Prepare the matcher

	
	Mat grayA, grayB;
	cvtColor(*colorA, grayA, CV_RGB2GRAY);
	cvtColor(*colorB, grayB, CV_RGB2GRAY);

	RobustMatcher rmatcher;

	Ptr<FeatureDetector> pfd = new SurfFeatureDetector(10);

	rmatcher.setFeatureDetector(pfd);
	// Match the two images
	vector<DMatch> matches;
	vector<KeyPoint> keypointsA, keypointsB;
	Mat fundemental= rmatcher.match(grayA, grayB, matches, keypointsA, keypointsB);
	

	Mat img_matches;
	drawMatches( grayA, keypointsA, grayB, keypointsB,
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  
    //-- Show detected matches
    imshow( "Good Matches", img_matches );

	vector<Point2f> selPoints1, selPoints2;
	KeyPoint::convert(keypointsA, selPoints1);
	KeyPoint::convert(keypointsB, selPoints2);

	vector<Point3f *> setA, setB;
	for(int i = 0; i < matches.size(); i++)
	{
		Point2f point2D = selPoints1.at(matches.at(i).queryIdx);
		int bit0 = depthA->at<Vec3b>(point2D.y, point2D.x)[0];
		int bit1 = depthA->at<Vec3b>(point2D.y, point2D.x)[1];
		double depth = (bit0 | bit1 << 8 );
		Point3f *point3D = new Point3f(point2D.x, point2D.y, depth / 100); 
		setA.push_back(point3D);

		point2D = selPoints2.at(matches.at(i).trainIdx);
		bit0 = depthB->at<Vec3b>(point2D.y, point2D.x)[0];
		bit1 = depthB->at<Vec3b>(point2D.y, point2D.x)[1];
		depth = (bit0 | bit1 << 8 );
		Point3f *point3DB = new Point3f(point2D.x, point2D.y, depth / 100); 
		setB.push_back(point3DB);
	}


	Transformation result(false);
	HornMethod hornMethod;
	hornMethod.getTransformation(setA, setB, &result);
	inverse->invert(&result);
	// clean vectors SetA and SetB
	
}




int main (int argc, char **argv) {
 
	CvCapture* captureDepth = cvCaptureFromAVI("depth.avi");
	CvCapture* captureColor = cvCaptureFromAVI("color.avi");

	for(int i = 0; i < START_FRAME; i++ )
	{
		cvQueryFrame(captureColor);
		cvQueryFrame(captureDepth);
	}
	
	IplImage* img = 0; 
	img = cvQueryFrame(captureColor);	//retrieve the captured frame
	Mat imgA(img,true);
	img = cvQueryFrame(captureDepth);	//retrieve the captured frame
	Mat depthA(img,true);

	int frameNo = START_FRAME;
	Transformation *global = new Transformation(true);
	Visualizer::init(200,150, 600, 0, 0, 0);
	Visualizer::addImageFrame(&imgA, &depthA);
	while(frameNo <= LAST_FRAME)
	{
		
		for(int j = 0; j < STEP; j++)
		{
			cvQueryFrame(captureColor);
			cvQueryFrame(captureDepth);
		}

		img = cvQueryFrame(captureColor);	//retrieve the captured frame
		Mat imgB(img,true);
		Mat depthB = cvQueryFrame(captureDepth);

	    Transformation *inverse = new Transformation(false);
		alignFrames(&imgA, &imgB, &depthA, &depthB, inverse);
		int noPixels = 240 * 320;
		GLfloat *vertices = new GLfloat[noPixels * 3];
		GLfloat *colors = new GLfloat[noPixels * 3];
		global->concatenate(inverse);
		global->applyToFrame(&imgB, &depthB, vertices, colors);
		Visualizer::addPoints(vertices, colors, noPixels);
		
		imgB.copyTo(imgA);
		depthB.copyTo(depthA);
		
		frameNo += STEP;
		break;
	}

	Visualizer::start(argc, argv);
	
    return 0;
}
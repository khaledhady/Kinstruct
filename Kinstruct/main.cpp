#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Visualizer.h"
#include "RobustMatcher.h"
#include "AbsoluteOrientation.h"
//#include "Transformation.h"
#include "Constructor.h"
using namespace cv;
using namespace std;

/* Using the GLUT library for the base windowing setup */
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>

#define FORWARD_STEP_SCALE 50
#define BACKWARD_STEP_SCALE 50
#define RIGHT_STEP_SCALE 10
#define LEFT_STEP_SCALE 10
#define UP_STEP_SCALE 50
#define DOWN_STEP_SCALE 50

//angle of rotation
//float xpos = 200, ypos = 150, zpos = 600, xrot = 0, yrot = 0, angle=0.0;
//float lastx, lasty;

int main (int argc, char **argv) {
 
	CvCapture* captureDepth = cvCaptureFromAVI("depth.avi");
	CvCapture* captureColor = cvCaptureFromAVI("color.avi");
	for(int j = 0; j < 100; j++)
	{
		cvQueryFrame(captureColor);
		cvQueryFrame(captureDepth);
	}
	/*cvQueryFrame(captureColor);
	cvQueryFrame(captureDepth);*/
	IplImage* img = 0; 
	int nFrames = 50;
	int i = 0 ;
	img = cvQueryFrame(captureColor);	//retrieve the captured frame

	//IplImage * depthImage = cvRetrieveFrame(captureDepth);

	IplImage* gray1 = cvCreateImage( cvGetSize( img ), 8, 1 ); 
	cvCvtColor(img,gray1,CV_BGR2GRAY);
	Mat img1(gray1,true);
	Mat imgA(img,true);

	img = cvQueryFrame(captureDepth);	//retrieve the captured frame
	
	Mat depthA(img,true);


	for(int j = 0; j < 50; j++)
	{
		cvQueryFrame(captureColor);
		cvQueryFrame(captureDepth);
	}
	img = cvQueryFrame(captureColor);	//retrieve the captured frame
	IplImage* gray2 = cvCreateImage( cvGetSize( img ), 8, 1 ); 
	cvCvtColor(img,gray2,CV_BGR2GRAY);
	Mat img2(gray2,true);
	Mat imgB(img,true);

	
	Mat depthB = cvQueryFrame(captureDepth);
	
	
	// Prepare the matcher
RobustMatcher rmatcher;

cv::Ptr<cv::FeatureDetector> pfd=new cv::SurfFeatureDetector(10);

rmatcher.setFeatureDetector(pfd);
// Match the two images
std::vector<cv::DMatch> matches;
std::vector<cv::KeyPoint> keypoints1, keypoints2;
cv::Mat fundemental= rmatcher.match(img1,img2,
matches, keypoints1, keypoints2);
std::vector<cv::KeyPoint> points1, points2;
cv::Scalar color= cv::Scalar(255,255,255);

 Mat img_matches;
  drawMatches( gray1, keypoints1, gray2, keypoints2,
               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  
  //-- Show detected matches
  imshow( "Good Matches", img_matches );


//cout << keypoints1.size() << endl;
//cout << keypoints2.size() << endl;
// for all corners

std::vector<cv::Point2f> selPoints1, selPoints2;
cv::KeyPoint::convert(keypoints1,selPoints1);
cv::KeyPoint::convert(keypoints2,selPoints2);

std::vector<cv::Point3f *> setAmine, setBmine;
for(int i = 0; i < matches.size(); i++)
{

	Point2f point2D = selPoints1.at(matches.at(i).queryIdx);
	int bit0 = depthA.at<cv::Vec3b>(point2D.y, point2D.x)[0];
	int bit1 = depthA.at<cv::Vec3b>(point2D.y, point2D.x)[1];
	double depth = (bit0 | bit1 << 8 );
	Point3f *point3D = new Point3f(point2D.x, point2D.y, depth / 100); 
	setAmine.push_back(point3D);

	point2D = selPoints2.at(matches.at(i).trainIdx);
	bit0 = depthB.at<cv::Vec3b>(point2D.y, point2D.x)[0];
	bit1 = depthB.at<cv::Vec3b>(point2D.y, point2D.x)[1];
	depth = (bit0 | bit1 << 8 );
	Point3f *point3DB = new Point3f(point2D.x, point2D.y, depth / 100); 
	setBmine.push_back(point3DB);
	cout << i << endl;
}
cv::circle(img1,
cv::Point(100, 5),
5,cv::Scalar(255,255,0));
std::vector<Point3D> setA, setB;
 cv::namedWindow("Image");
		cv::imshow("Image", img1);
        // Wait for the user to press a key in the GUI window.
        
		cv::namedWindow("Image2");
		cv::imshow("Image2", img2);
for(int i = 0; i < matches.size(); i++)
{

	Point2f point2D = selPoints1.at(matches.at(i).queryIdx);
	double dim[3];
	int bit0 = depthA.at<cv::Vec3b>(point2D.y, point2D.x)[0];
	int bit1 = depthA.at<cv::Vec3b>(point2D.y, point2D.x)[1];
	double depth = (bit0 | bit1 << 8 );
	dim[0] = point2D.x;   dim[1] = point2D.y;   dim[2] = depth / 100; 
	Point3D point3D(dim); 
	setA.push_back(point3D);

	

	point2D = selPoints2.at(matches.at(i).trainIdx);
	
	bit0 = depthB.at<cv::Vec3b>(point2D.y, point2D.x)[0];
	bit1 = depthB.at<cv::Vec3b>(point2D.y, point2D.x)[1];
	depth = (bit0 | bit1 << 8 );
	dim[0] = point2D.x;   dim[1] = point2D.y;   dim[2] = depth / 100; 
	Point3D point3DB(dim); 
	
	setB.push_back(point3DB);
	cout << i << endl;
}

AbsoluteOrientation absolute;
Frame t;
absolute.compute(setA, setB, t);
cout << t;
t.invert();

Transformation result;
Constructor construct;
//construct.selfTest();
construct.getTransformation(setAmine, setBmine, &result);




	Transformation inverse;
	inverse.invert(&result);
	



std::vector<cv::Point2f>::const_iterator it=selPoints1.begin();


std::vector<cv::Point2f>::const_iterator it2=selPoints2.begin();

       
		

		

	Visualizer::init(200,150, 600, 0, 0, 0);
	int noPixels = 240 * 320;
	GLfloat *vertices = new GLfloat[noPixels * 3];
		GLfloat *colors = new GLfloat[noPixels * 3];
			
		
		int pixelIndex = 0;
	for(int i = 0; i < 1; i++)
	{
		/*Mat imageRGB = cvQueryFrame(captureColor);
	Mat imageDepth = cvQueryFrame(captureDepth);*/
		

	
	


	GLfloat *vertices = new GLfloat[noPixels * 3];
		GLfloat *colors = new GLfloat[noPixels * 3];
			
		int rowsRGB = imgA.rows;
		int colsRGB = imgA.cols;
		int pixelIndex = 0;
		for (int k = 0; k < rowsRGB ; k++) {
			
			for (int m = 0; m < colsRGB; m++) {
			
				int bit0 = depthA.at<cv::Vec3b>(k,m)[0];
				int bit1 = depthA.at<cv::Vec3b>(k,m)[1];
				int depth = (bit0 | bit1 << 8 );
				//depth = (int)(bit0 >> 3 | bit1 << 5);
				

				double blue = imgA.at<cv::Vec3b>(k,m)[0];
				double green = imgA.at<cv::Vec3b>(k,m)[1];
				double red = imgA.at<cv::Vec3b>(k,m)[2];

				vertices[pixelIndex] = m;
				vertices[pixelIndex + 1] = -k;
				vertices[pixelIndex + 2] = depth / 100;

				colors[pixelIndex] = red / 255;
				colors[pixelIndex + 1] = green / 255;
				colors[pixelIndex + 2] = blue / 255;
				
			
				pixelIndex += 3;
			
			}
		}
		Visualizer::addPoints(vertices, colors, noPixels);
		
	}
	pixelIndex = 0;
		for (int k = 0; k < 240 ; k++) {
			
			for (int m = 0; m < 320; m++) {

				int bit0 = depthB.at<cv::Vec3b>(k,m)[0];
				int bit1 = depthB.at<cv::Vec3b>(k,m)[1];
				int depth = (bit0 | bit1 << 8 );
				//depth = (int)(bit0 >> 3 | bit1 << 5);

				double blue = imgB.at<cv::Vec3b>(k,m)[0];
				double green = imgB.at<cv::Vec3b>(k,m)[1];
				double red = imgB.at<cv::Vec3b>(k,m)[2];
				double dim[3];
				dim[0] = m; dim[1] = k; dim[2] = depth / 100;
				Point3D ok(dim);
				Point3d myPoint(m, k, depth / 100);
				//inverse.applyToPoint(&ok);
				//t.apply(ok);
				inverse.applyToPoint(&myPoint);
				/*vertices[pixelIndex] = ok[0];
				vertices[pixelIndex + 1] = -ok[1];
				vertices[pixelIndex + 2] = ok[2];*/
				vertices[pixelIndex] = myPoint.x;
				vertices[pixelIndex + 1] = -myPoint.y;
				vertices[pixelIndex + 2] = myPoint.z;

				colors[pixelIndex] = red / 255;
				colors[pixelIndex + 1] = green / 255;
				colors[pixelIndex + 2] = blue / 255;
				pixelIndex += 3;
			}
		}
		Visualizer::addPoints(vertices, colors, noPixels);
	
	Visualizer::start(argc, argv);
	
	


    return 0;
}
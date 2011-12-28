#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "Visualizer.h"
#include "RobustMatcher.h"
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
float xpos = 200, ypos = 150, zpos = 600, xrot = 0, yrot = 0, angle=0.0;
float lastx, lasty;

Mat captureRGBFrame()
{
	CvCapture* capture = cvCaptureFromAVI("color.avi");
	int isColor = 1;
	int fps     = 25;  // or 30
	int frameW  = 480; // 744 for firewire cameras
	int frameH  = 240; // 480 for firewire cameras
	IplImage* img = 0; 
	int nFrames = 50;
	int i = 0 ;
	
	cvQueryFrame(capture); // this call is necessary to get correct 
	// capture properties
	int frameH2    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
	int frameW2    = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	int fps2       = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);

	//namedWindow("matWindow");
	//while(1)
	//{
		if(!cvGrabFrame(capture)){              // capture a frame 
			printf("Could not grab a frame\n\7");
			exit(0);
		}

		img = cvRetrieveFrame(capture);	//retrieve the captured frame
		Mat hamda(img,true);
		
		//imshow("matWindow",hamda);

		//cvShowImage("mainWin", img);
		//int c = cvWaitKey(1);

		//if( c == 27 || c == 'q' || c == 'Q' )

		//	break;
	//}
		return hamda; 
}

Mat captureDepthFrame()
{
	CvCapture* capture = cvCaptureFromAVI("depth.avi");
	
	IplImage* img = 0; 
	int nFrames = 50;
	int i = 0 ;
	
	cvQueryFrame(capture); // this call is necessary to get correct 
	// capture properties
	

	//namedWindow("matWindow");
	//while(1)
	//{
		if(!cvGrabFrame(capture)){              // capture a frame 
			printf("Could not grab a frame\n\7");
			exit(0);
		}

		img = cvRetrieveFrame(capture);	//retrieve the captured frame
		Mat hamda(img,true);
		
		//imshow("matWindow",hamda);

		//cvShowImage("mainWin", img);
		//int c = cvWaitKey(1);

		//if( c == 27 || c == 'q' || c == 'Q' )

		//	break;
	//}
		return hamda;
}


int main (int argc, char **argv) {
 
	CvCapture* captureDepth = cvCaptureFromAVI("depthF.avi");
	CvCapture* captureColor = cvCaptureFromAVI("colorF.avi");
	cvQueryFrame(captureColor);
	IplImage* img = 0; 
	int nFrames = 50;
	int i = 0 ;
	img = cvRetrieveFrame(captureColor);	//retrieve the captured frame
	IplImage* gray1 = cvCreateImage( cvGetSize( img ), 8, 1 ); 
	cvCvtColor(img,gray1,CV_BGR2GRAY);
	Mat img1(gray1,true);
	for(int j = 0; j < 130; j++)
	{
		cvQueryFrame(captureColor);
		cvQueryFrame(captureDepth);
		
	}
	img = cvRetrieveFrame(captureColor);	//retrieve the captured frame
	IplImage* gray2 = cvCreateImage( cvGetSize( img ), 8, 1 ); 
	cvCvtColor(img,gray2,CV_BGR2GRAY);
	Mat img2(gray2,true);
	
	
	
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


cout << keypoints1.size() << endl;
cout << keypoints2.size() << endl;
// for all corners

std::vector<cv::Point2f> selPoints1, selPoints2;
cv::KeyPoint::convert(keypoints1,selPoints1);
cv::KeyPoint::convert(keypoints2,selPoints2);

std::vector<cv::Point2f>::const_iterator it=selPoints1.begin();


std::vector<cv::Point2f>::const_iterator it2=selPoints2.begin();

        cv::namedWindow("Image");
		cv::imshow("Image", img1);
        // Wait for the user to press a key in the GUI window.
        
		cv::namedWindow("Image2");
		cv::imshow("Image2", img2);
		

		

	Visualizer::init(200,150, 600, 0, 0, 0);

	for(int i = 0; i < 1; i++)
	{
		Mat imageRGB = cvQueryFrame(captureColor);
	Mat imageDepth = cvQueryFrame(captureDepth);
		

	
	


	int noPixels = imageRGB.rows * imageRGB.cols;
	GLfloat *vertices = new GLfloat[noPixels * 3];
		GLfloat *colors = new GLfloat[noPixels * 3];
			
		int rowsRGB = imageRGB.rows;
		int colsRGB = imageRGB.cols;
		int pixelIndex = 0;
		for (int k = 0; k < rowsRGB ; k++) {
			
			for (int m = 0; m < colsRGB; m++) {
			
				int bit0 = imageDepth.at<cv::Vec3b>(k,m)[0];
				int bit1 = imageDepth.at<cv::Vec3b>(k,m)[1];
				int depth = (bit0 | bit1 << 8 );

				double blue = imageRGB.at<cv::Vec3b>(k,m)[0];
				double green = imageRGB.at<cv::Vec3b>(k,m)[1];
				double red = imageRGB.at<cv::Vec3b>(k,m)[2];

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
		
	
	Visualizer::start(argc, argv);
	
	


    return 0;
}
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "Visualizer.h"
#include "RobustMatcher.h"
#include "HornMethod.h"


#include <iostream>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#define START_FRAME 300
#define STEP 5
#define LAST_FRAME 320
using namespace cv;
using namespace std;
using namespace pcl;
int window = 0;

/* Using the GLUT library for the base windowing setup */
#include <GL/glut.h>
#include <GL/gl.h>
#include <math.h>
#include <time.h>



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
	cout << "getting matches \n";
	time_t before;

    before = time (NULL);
    
  
	Mat fundemental= rmatcher.match(grayA, grayB, matches, keypointsA, keypointsB);
	time_t after;

	after = time (NULL);
	
  
	cout << "Found matches " << after - before << endl;
	

	//Mat img_matches;
	//drawMatches( grayA, keypointsA, grayB, keypointsB,
      //         matches, img_matches, Scalar::all(-1), Scalar::all(-1),
        //       vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  
    //-- Show detected matches
   // imshow( "Good Matches" + window, img_matches );
	//window++;

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
		Point3f *point3D = new Point3f(point2D.x, point2D.y, bit0 * 3500 / 2550); 
		setA.push_back(point3D);

		point2D = selPoints2.at(matches.at(i).trainIdx);
		bit0 = depthB->at<Vec3b>(point2D.y, point2D.x)[0];
		bit1 = depthB->at<Vec3b>(point2D.y, point2D.x)[1];
		depth = (bit0 | bit1 << 8 );
		Point3f *point3DB = new Point3f(point2D.x, point2D.y, bit0 * 3500 / 2550); 
		setB.push_back(point3DB);
	}

	cout << "getting transformation \n";
	before = time (NULL);
	Transformation result(false);
	HornMethod hornMethod;
	hornMethod.getTransformation(setA, setB, &result);
	inverse->invert(&result);
	after = time (NULL);
	
  
	cout << "Found transformation " << after - before << endl;
	// clean vectors SetA and SetB
	
}


void addPoints(Mat *color, Mat *depth, Transformation *transformation, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	int pixelIndex = 0;
	cloud->points.resize (color->rows * color->cols);

	for (int k = 0; k < color->rows ; k++) {
		for (int m = 0; m < color->cols; m++) {
			
			int bit0 = depth->at<cv::Vec3b>(k,m)[0];
			//int bit1 = depth->at<cv::Vec3b>(k,m)[1];
			//double z = (bit0 | bit1 << 8 );
				
			int blue = color->at<cv::Vec3b>(k,m)[0];
			int green = color->at<cv::Vec3b>(k,m)[1];
			int red = color->at<cv::Vec3b>(k,m)[2];
			
			

			Point3d myPoint(m, k, bit0 * 3500 / 2550);
			transformation->applyToPoint(&myPoint);

			cloud->points[pixelIndex].x = myPoint.x;
			cloud->points[pixelIndex].y = myPoint.y;
			cloud->points[pixelIndex].z = myPoint.z;
			uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
              static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
			cloud->points[pixelIndex].rgb = *reinterpret_cast<float*>(&rgb);
			
			pixelIndex++;

			
		}
	}
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


	//pcl::PointCloud<pcl::PointXYZRGB> *cloud = new pcl::PointCloud<pcl::PointXYZRGB>();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	int noPixels = imgA.rows * imgA.cols;
	
		
	// Fill in the cloud data
  cloud->width    = 320;
  cloud->height   = 240;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
	int rowsRGB = imgA.rows;
	int colsRGB = imgA.cols;
	int o = 0;
	for (int k = 0; k < rowsRGB ; k++) {
		for (int m = 0; m < colsRGB; m++) {
			int bit0 = depthA.at<cv::Vec3b>(k,m)[0];
			int bit1 = depthA.at<cv::Vec3b>(k,m)[1];
			int depth = (bit0 | bit1 << 8 );
			int blue = imgA.at<cv::Vec3b>(k,m)[0];
			int green = imgA.at<cv::Vec3b>(k,m)[1];
			int red = imgA.at<cv::Vec3b>(k,m)[2];
			cloud->points[o].x = m;
			cloud->points[o].y = k;
			cloud->points[o].z = bit0 * 3500 / 2550;
			uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
              static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
			cloud->points[o].rgb = *reinterpret_cast<float*>(&rgb);
			
			o++;
				
			
		}
	}
	 
  
	/*pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);*/
    //blocks until the cloud is actually rendered
    
  /*pcl::io::savePCDFileASCII ("test.pcd", cloud);
  cout << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;*/

  

	int frameNo = START_FRAME;
	Transformation *global = new Transformation(true);
	/*Visualizer::init(200,150, 600, 0, 0, 0);
	Visualizer::addImageFrame(&imgA, &depthA);*/
	while(frameNo <= LAST_FRAME)
	{
		
		for(int j = 0; j < STEP; j++)
		{
			cvQueryFrame(captureColor);
			cvQueryFrame(captureDepth);
		}

		img = cvQueryFrame(captureColor);	//retrieve the captured frame
		Mat imgB(img, true);
		img = cvQueryFrame(captureDepth);
		Mat depthB(img, true);

	    Transformation *inverse = new Transformation(false);
		alignFrames(&imgA, &imgB, &depthA, &depthB, inverse);
		int noPixels = 240 * 320;
		/*GLfloat *vertices = new GLfloat[noPixels * 3];
		GLfloat *colors = new GLfloat[noPixels * 3];*/
		//global->concatenate(inverse);
		//global->applyToFrame(&imgB, &depthB, vertices, colors);
		//Visualizer::addPoints(vertices, colors, noPixels);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
		addPoints(&imgB, &depthB, inverse, tmp);
		*cloud += *tmp;
		//imgB.copyTo(imgA);
		//depthB.copyTo(depthA);
		
		frameNo += STEP;
		//break;
	}
	
    
	 /*pcl::io::savePCDFileASCII ("test.pcd", *cloud);
  cout << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;*/
	//Visualizer::start(argc, argv);

	 pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
    
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    }
	
	
    return 0;
}
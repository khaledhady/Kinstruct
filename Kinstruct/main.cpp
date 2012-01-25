#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "RobustMatcher.h"
#include "HornMethod.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/point_cloud.h>
#define START_FRAME 300
#define STEP 10
#define LAST_FRAME 350
//using namespace cv;
using namespace std;
using namespace pcl;
int window = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
std::stringstream compressedData;

/* Using the GLUT library for the base windowing setup */
#include <GL/glut.h>
#include <GL/gl.h>
#include <math.h>
#include <time.h>

#include <boost/thread.hpp>  
#include <boost/date_time.hpp>  
   bool update;
   boost::mutex updateModelMutex;

void alignFrames(cv::Mat *colorA, cv::Mat *colorB, cv::Mat *depthA, cv::Mat *depthB, Transformation *inverse)
{
	// Prepare the matcher
	cv::Mat grayA, grayB;
	cv::cvtColor(*colorA, grayA, CV_RGB2GRAY);
	cv::cvtColor(*colorB, grayB, CV_RGB2GRAY);

	RobustMatcher rmatcher;

	cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector(10);

	rmatcher.setFeatureDetector(pfd);
	// Match the two images
	vector<cv::DMatch> matches;
	vector<cv::KeyPoint> keypointsA, keypointsB;
	cout << "getting matches \n";
	time_t before;
    before = time (NULL);
    
  
	cv::Mat fundemental= rmatcher.match(grayA, grayB, matches, keypointsA, keypointsB);
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

	vector<cv::Point2f> selPoints1, selPoints2;
	cv::KeyPoint::convert(keypointsA, selPoints1);
	cv::KeyPoint::convert(keypointsB, selPoints2);

	vector<cv::Point3f> setA, setB;
	for(int i = 0; i < matches.size(); i++)
	{
		cv::Point2f point2D = selPoints1.at(matches.at(i).queryIdx);
		int bit0 = depthA->at<cv::Vec3b>(point2D.y, point2D.x)[0];
		int bit1 = depthA->at<cv::Vec3b>(point2D.y, point2D.x)[1];
		double depth = (bit0 | bit1 << 8 );
		cv::Point3f point3D (point2D.x, point2D.y, bit0 * 3500 / 2550); 
		setA.push_back(point3D);

		point2D = selPoints2.at(matches.at(i).trainIdx);
		bit0 = depthB->at<cv::Vec3b>(point2D.y, point2D.x)[0];
		bit1 = depthB->at<cv::Vec3b>(point2D.y, point2D.x)[1];
		depth = (bit0 | bit1 << 8 );
		cv::Point3f point3DB (point2D.x, point2D.y, bit0 * 3500 / 2550); 
		setB.push_back(point3DB);
	}

	cout << "getting transformation \n";
	before = time (NULL);
	Transformation result(false);
	HornMethod hornMethod;
	hornMethod.getTransformation(&setA, &setB, &result);
	inverse->invert(&result);
	after = time (NULL);
	cout << "Found transformation " << after - before << endl;
	// clean vectors SetA and SetB
	
}

   
void visualize()  
{  
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->resetCameraViewpoint("sample cloud");
	pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
	PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	  
	
		// output pointcloud
		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());*/
		boost::mutex::scoped_lock updateLock(updateModelMutex);
		if(update)
		{
			viewer->updatePointCloud(cloud, "sample cloud");
			update = false;
		}
		updateLock.unlock();
     
		// decompress point cloud
	  
		// PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}   
    std::cout << "Worker: finished" << std::endl;  
}  
   

int main () {
 
	CvCapture* captureDepth = cvCaptureFromAVI("depth.avi");
	CvCapture* captureColor = cvCaptureFromAVI("color.avi");

	for(int i = 0; i < START_FRAME; i++ )
	{
		cvQueryFrame(captureColor);
		cvQueryFrame(captureDepth);
	}
	
	IplImage* img = 0; 
	img = cvQueryFrame(captureColor);	//retrieve the captured frame
	cv::Mat imgA(img,true);
	img = cvQueryFrame(captureDepth);	//retrieve the captured frame
	cv::Mat depthA(img,true);


	int noPixels = imgA.rows * imgA.cols;
	
	
  /*pcl::io::savePCDFileASCII ("test.pcd", cloud);
  cout << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;*/

  

	int frameNo = START_FRAME;
	Transformation *global = new Transformation(true);
	global->applyToFrame(&imgA, &depthA, cloud);
	boost::thread workerThread(visualize); 
	
   pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
  
  pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;

    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (compressionProfile, false);
	// stringstream to store compressed point cloud
      /*std::stringstream compressedData;*/
	while(frameNo <= LAST_FRAME)
	{
		
		for(int j = 0; j < STEP; j++)
		{
			cvQueryFrame(captureColor);
			cvQueryFrame(captureDepth);
		}

		img = cvQueryFrame(captureColor);	//retrieve the captured frame
		cv::Mat imgB(img, true);
		img = cvQueryFrame(captureDepth);
		cv::Mat depthB(img, true);

	    Transformation *inverse = new Transformation(false);
		alignFrames(&imgA, &imgB, &depthA, &depthB, inverse);
		//global->concatenate(inverse);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);

		inverse->applyToFrame(&imgA, &depthA, tmp);
		boost::mutex::scoped_lock updateLock(updateModelMutex); // defer_lock makes it initially unlocked
			update = true;
			*cloud += *tmp;
		updateLock.unlock();
		
		// compress point cloud
        // PointCloudEncoder->encodePointCloud (cloud, compressedData);
	    // cout << "Compressed" << endl;
		//imgB.copyTo(imgA);
		//depthB.copyTo(depthA);
		frameNo += STEP;
		//viewer->spinOnce(1000);
		//break;
	}
	
	/*viewer->spinOnce(100);

	}*/
	
    
	 /*pcl::io::savePCDFileASCII ("test.pcd", *cloud);
  cout << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;*/
	//Visualizer::start(argc, argv);

	
  

	   delete (PointCloudEncoder);
  
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	/*pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);*/
	cout << "start filtering" << endl;
	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);*/
  cout << "done filtering" << endl;
       
    
	  workerThread.join();  
       
    std::cout << "main: done" << std::endl;  
	
	
    return 0;
}


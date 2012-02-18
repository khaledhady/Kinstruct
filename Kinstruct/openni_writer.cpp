#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>


#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif

boost::mutex saveCloud;

#include "Commons.h"
#include "RobustMatcher.h"
#include "HornMethod.h"
#include <math.h>
#include <time.h>
#include <boost/thread.hpp>  
#include <boost/date_time.hpp>  
#include "pcl/win32_macros.h"
#define START_FRAME 200
#define STEP 10
#define LAST_FRAME 300
//using namespace cv;
using namespace std;
using namespace pcl;
int noFrames = 0;
bool first = true;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr onlineView (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultColored (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastAligned (new pcl::PointCloud<pcl::PointXYZRGB>);
Eigen::Matrix4f lastTransformation;
std::stringstream compressedData;
bool updateOnline;
bool updateBuilt;
bool capturedNew;
boost::mutex updateModelMutex;
boost::mutex updateOnlineMutex;
boost::mutex updateCloudBMutex;
bool stop;
bool start;
void cloudToMat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat *color, cv::Mat *depth)
{
	for(int i = 0; i < color->cols; i++)
	{

		for(int j = 0; j < color->rows; j++)
		{
			pcl::PointXYZRGB point = cloud->at(i + 30, j + 30);
			   if (!isFinite (point))
			        continue;

			color->at<cv::Vec3b>(j, i)[0] = point.b;
			color->at<cv::Vec3b>(j, i)[1] = point.g;
			color->at<cv::Vec3b>(j, i)[2] = point.r;
			depth->at<float>(j, i) = point.z;
			

		}
	}
}

bool alignFrames(cv::Mat *colorA, cv::Mat *colorB, cv::Mat *depthA, cv::Mat *depthB, Transformation *inverse,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB)
{
	RobustMatcher rmatcher;
	vector<cv::DMatch> matches;
	vector<cv::KeyPoint> keypointsA, keypointsB;
	cout << "Tracking.. " << endl ;
	time_t before;
    before = time (NULL);
	cv::Mat imageA, imageB;
	colorA->copyTo(imageA);
	colorB->copyTo(imageB);
    int tracked = rmatcher.track(imageA, imageB);
	
	time_t after;

	after = time (NULL);
	cout << "Finished Tracking " << after - before << endl;

	cv::imshow( "Current keyframe " , imageA);
	cv::imshow( "Online " , imageB );
	cv::waitKey(30);

	if(tracked > 250 || tracked < 50)
		return true;


	vector<cv::Point2f> selPoints1, selPoints2;
	cv::KeyPoint::convert(keypointsA, selPoints1);
	cv::KeyPoint::convert(keypointsB, selPoints2);

	vector<cv::Point3f> setA, setB;

	for(int i = 0; i < rmatcher.initial.size() ; i++)
	{
		cv::Point2f point2DA = rmatcher.initial.at(i);
		cv::Point2f point2DB = rmatcher.final.at(i);
		/*cout << point2DA.x << point2DA.y << endl;
		cout << point2DB.x << point2DB.y << endl;*/
		if(point2DA.x < 0 || point2DA.y < 0 || point2DB.x < 0 || point2DB.y < 0)
			continue;
		pcl::PointXYZRGB pointA = cloudA->at(point2DA.x, point2DA.y);
		pcl::PointXYZRGB pointB = cloudB->at(point2DB.x, point2DB.y);
		
		  if (!isFinite (pointA) || !isFinite (pointB))
			    continue;
		  //cout << pointA.z << endl; 

		cv::Point3f point3D (pointA.x, pointA.y, pointA.z); 
		setA.push_back(point3D);
		cv::Point3f point3DB (pointB.x, pointB.y, pointB.z); 
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
	return false;

}

void makeRed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
{
	for(int i = 0; i < cloudToColor->size(); i++)
	{
		cloudToColor->points[i].r = 255;
		cloudToColor->points[i].g = 0;
		cloudToColor->points[i].b = 0;
	}
}

void makeGreen(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
{
	for(int i = 0; i < cloudToColor->size(); i++)
	{
		cloudToColor->points[i].r = 0;
		cloudToColor->points[i].g = 255;
		cloudToColor->points[i].b = 0;
	}
}

void makeBlue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
{
	for(int i = 0; i < cloudToColor->size(); i++)
	{
		cloudToColor->points[i].r = 0;
		cloudToColor->points[i].g = 0;
		cloudToColor->points[i].b = 255;
	}
}

void makeYellow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
{
	for(int i = 0; i < cloudToColor->size(); i++)
	{
		cloudToColor->points[i].r = 255;
		cloudToColor->points[i].g = 255;
		cloudToColor->points[i].b = 0;
	}
}

void makeWhite(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
{
	for(int i = 0; i < cloudToColor->size(); i++)
	{
		cloudToColor->points[i].r = 255;
		cloudToColor->points[i].g = 255;
		cloudToColor->points[i].b = 255;
	}
}

void downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled)
{
  // Create the filtering object
  std::cerr << "PointCloud before filtering: " << original->width * original->height 
       << " data points (" << pcl::getFieldsList (*original) << ").";
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (original);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*downsampled);
  std::cerr << "PointCloud after filtering: " << downsampled->width * downsampled->height 
       << " data points (" << pcl::getFieldsList (*downsampled) << ").";
}

void main1()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredA (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledA (new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::mutex::scoped_lock initializeLock(updateModelMutex);
	updateBuilt = true;
	*result += *cloudA;
	downsample(cloudA, lastAligned);
	initializeLock.unlock();
	cv::Mat imgA(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
	cv::Mat depthA(cloudA->height - 50, cloudA->width - 50, CV_32F );
	pcl::PassThrough<pcl::PointXYZRGB> pass; 
	pass.setInputCloud( cloudA );
	pass.filter( *coloredA );
	cloudToMat(cloudA, &imgA, &depthA);
	int times = 1;
	bool first = true;
	while(!stop)
	{
		boost::mutex::scoped_lock updateCloudBLock(updateCloudBMutex);
		if(capturedNew)
		{
			cout << "started" << endl;
			times++;
			//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredB (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledB (new pcl::PointCloud<pcl::PointXYZRGB>);

			cv::Mat imgB(cloudB->height - 50, cloudB->width - 50, CV_8UC3 );
			cv::Mat depthB(cloudB->height - 50, cloudB->width - 50, CV_32F );

			pass.setInputCloud( cloudB );
			pass.filter( *coloredB );

			cloudToMat(cloudB, &imgB, &depthB);
			//cv::waitKey(0);
			Transformation inverse(false);
			bool tooClose = alignFrames(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB);
			if(!tooClose)
			{
				
				
				//cv::waitKey(0);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedDownsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
				//if(times == 2)
					//global->concatenate(&inverse);
				cout << "getting ICP \n";
				time_t before;
				before = time (NULL);
				Eigen::Matrix4f initialTransformation;
				if(!first)
					inverse.concatenate(&lastTransformation);
				first = false;
				inverse.get4X4Matrix(&initialTransformation);
				pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
				//boost::mutex::scoped_lock downsamplingLock(updateModelMutex);
				//downsample(result, downsampledA);
				//downsamplingLock.unlock();
				downsample(coloredB, downsampledB);
				icp.setInputCloud(downsampledB);
				icp.setInputTarget(lastAligned);
  
				icp.setMaxCorrespondenceDistance(0.1);
				//icp.setRANSACOutlierRejectionThreshold(0.05);
				//icp.setTransformationEpsilon(0.000001);
				icp.setEuclideanFitnessEpsilon(0.0001);
				//icp.setMaximumIterations(1000);
				icp.align(*transformedDownsampled, initialTransformation);
				lastAligned->clear();
				pcl::copyPointCloud(*transformedDownsampled, *lastAligned);
				std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
				//std::cout << icp.getFinalTransformation() << std::endl;
				time_t after = time (NULL);
				cout << "Found Transformation " << after - before << endl;
				lastTransformation = icp.getFinalTransformation();
				//global->concatenate(&icp.getFinalTransformation());
				//Eigen::Matrix4f finalTransformation;
				//global->get4X4Matrix(&finalTransformation);
				boost::mutex::scoped_lock updateLock(updateModelMutex);
				updateBuilt = true;
				result->clear();
				pcl::transformPointCloud(*coloredB, *result, icp.getFinalTransformation());
				*global += *result;
				resultColored->clear();
				pcl::copyPointCloud(*result, *transformed);
				//detectNew(result, transformed);
				//*result += *transformed;
			/*
				if(times == 2)
				{	
					makeRed(coloredA);
					makeGreen(transformed);
					*resultColored += *coloredA;
				}else if(times == 3)
				{
					makeBlue(transformed);
				}else if(times == 4)
				{
					makeYellow(transformed);
				}else if(times == 5)
				{
					makeWhite(transformed);
				}
		
				*resultColored += *transformed;*/
				updateLock.unlock();
				cloudA = cloudB;
				coloredA = coloredB;
				imgB.copyTo(imgA);
				depthB.copyTo(depthA);
			}
			else cout << "Frame is too close ... ignoring" << endl;
		}
		capturedNew = false;
		updateCloudBLock.unlock();
	}
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	  
	if (event.getKeySym () == "r" && event.keyDown ())
	{
		boost::mutex::scoped_lock saveLock(updateOnlineMutex);
		//std::cout << "r was pressed => removing all text" << std::endl;
		pcl::PassThrough<pcl::PointXYZRGB> pass; // can do this without parameters
		PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *onlineView ) );
		if(noFrames == 0)
		{
			cloudA = deep_copy;
			boost::thread workerThread(main1);
			cout << "Started recording --> " << endl;
			start = true;
		}
		else if(noFrames == 1){
			cout << "Obtained second Frame" <<endl ;
			cloudB = deep_copy;
			capturedNew = true;
			//boost::thread workerThread(main1);
			//workerThread.join();
			//main1();
		}else{
			cout << "Obtained "<< noFrames + 1 << "  Frame" << endl ;
			cloudA = cloudB;
			cloudB = deep_copy;
			capturedNew = true;
			//main1();
			//boost::thread workerThread(main1);
			//workerThread.join();
		}
		noFrames++;
		/*char fileName[20];
		itoa(noFrames, fileName, 10);*/

		std::string fileName = ".pcd";
		std::stringstream tmp;
		tmp << noFrames << ".pcd";
		
		//pcl::io::savePCDFileASCII(fileName, *globalcloud);
		//pcl::io::savePCDFileBinary(tmp.str() , *deep_copy);
		/*if(noFrames == 5)
			pcl::io::savePLYFile("room.ply", *result);*/
		saveLock.unlock();

	}else if (event.getKeySym () == "p" && event.keyDown ())
	{
		//boost::mutex::scoped_lock saLock(saveCloud);
		stop = true;
		//pcl::io::savePLYFileBi//("model.ply", *global);
		//saveLock.unlock();
		//pcl::io::savePLYFileBinary("model.ply", *global);

	}else if(event.getKeySym() == "s" && event.keyDown())
	{
		boost::mutex::scoped_lock saveLock(updateOnlineMutex);
		std::cout << "s was pressed => saving" << std::endl;
		PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *onlineView ) );
		std::stringstream tmp;
		tmp << noFrames << ".pcd";
		pcl::io::savePCDFileBinary(tmp.str() , *deep_copy);
		noFrames++;
		saveLock.unlock();
	}
}

	

class SimpleOpenNIViewer
{
public:
	int seconds;
	SimpleOpenNIViewer ()
	{
		seconds = 0;
	}
	


	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		seconds++;
		boost::mutex::scoped_lock updateOnlineLock(updateOnlineMutex);
		onlineView->clear();
		pcl::copyPointCloud(*cloud, *onlineView);
		//cout << "online is filled" << endl;
		updateOnline = true;
		

		PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *onlineView ) );
		updateOnlineLock.unlock();
		boost::mutex::scoped_lock updateCloudBLock(updateCloudBMutex);
		if(seconds % 10 == 0 && start && !stop)
		{
			
			/*if(noFrames == 1){
				cout << "Obtained second Frame" <<endl ;
			cloudB = deep_copy;
			capturedNew = true;
			}else{*/
			cout << "Obtained "<< noFrames + 1 << "  Frame" << endl ;
			/*cloudA = cloudB;*/
			cloudB = deep_copy;
			capturedNew = true;
		/*	}*/
			noFrames++;
		}
		updateCloudBLock.unlock();
	
		
		
		

	}

	void run ()
	{
		//pcl::Grabber* x = new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz, pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
		pcl::Grabber* x = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
		boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		x->registerCallback (f);

		//viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)&viewer);

		//viewer.registerKeyboardCallback(&SimpleOpenNIViewer::keyboardEventOccurred);
		x->start ();

		while(true) sleep(1);
		//while (!viewer.wasStopped())
		//{
		//	//sleep (1);
		//}

		x->stop ();
	}

	//pcl::visualization::CloudViewer viewer;
};

void visualize()  
{  
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor (0, 0, 0, v1);
	viewer->addText("Online", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(onlineView);
	viewer->addPointCloud<pcl::PointXYZRGB> (onlineView, rgb, "online", v1);
	viewer->registerKeyboardCallback(keyboardEventOccurred);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
	viewer->addText("Result in RGB", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbColored(result);
	viewer->addPointCloud<pcl::PointXYZRGB> (result, rgbColored, "result", v2);

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "online");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result");
	viewer->addCoordinateSystem (1.0);

	int i = 0;
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);


		boost::mutex::scoped_lock updateBuiltLock(updateModelMutex);
		/*if(updateLock.try_lock())*/
		
		if(updateBuilt)
		{
			stringstream tmp;
			tmp << i;
			//tmp << "color";
			viewer->addPointCloud<pcl::PointXYZRGB> (result, rgbColored, tmp.str(), v2);
			*global += *result;
			if(i == 0)
			{
				//viewer->resetCameraViewpoint(tmp.str());
			}
			updateBuilt = false;
			i++;
		}
		updateBuiltLock.unlock();

		boost::mutex::scoped_lock updateOnlineLock(updateOnlineMutex);
		if(updateOnline)
		{
			viewer->removePointCloud("online");
			viewer->addPointCloud<pcl::PointXYZRGB> (onlineView, rgb, "online", v1);
			//cout << "adding global" << endl;
			updateOnline = false;
		}
		updateOnlineLock.unlock();
		
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}   
    std::cout << "Worker: finished" << std::endl;  
}  

int main ()
{
	boost::thread workerThread(visualize); 
	SimpleOpenNIViewer v;
	v.run ();
    workerThread.join();  
	return 0;
}

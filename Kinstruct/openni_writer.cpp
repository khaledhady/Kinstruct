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


#include "Commons.h"
#include "Tracker.h"
#include "HornMethod.h"
#include "Alignment.h"
#include "SurfaceConstruction.h"
#include <math.h>
#include <time.h>
#include <boost/thread.hpp>  
#include <boost/date_time.hpp>  
#include "pcl/win32_macros.h"
using namespace std;
using namespace pcl;
int noFrames = 0;
Alignment alignment;
SurfaceConstruction surfaceConst;

// Global cloud that will hold the reconstructed model
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global (new pcl::PointCloud<pcl::PointXYZRGB>);

// Cloud holding the current feed from the Kinect camera
pcl::PointCloud<pcl::PointXYZRGB>::Ptr onlineView (new pcl::PointCloud<pcl::PointXYZRGB>);

// Clouds holding the two views to be aligned
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);

// This holds that last downsampled cloud aligned
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastAligned (new pcl::PointCloud<pcl::PointXYZRGB>);

// The global transformation
Eigen::Matrix4f globalTransformation = Eigen::Matrix4f::Identity ();

// to hold the constructed mesh
pcl::PolygonMesh triangles;

// Mutex to manage access to the constructed model
boost::mutex updateModelMutex;

// Mutex to manage access to the online view
boost::mutex updateOnlineMutex;

// Mutext to manage any read or write to the cloud that will be used for construction
boost::mutex updateCloudBMutex;
boost::condition_variable condQ;

// indicating any events
bool updateOnline;
bool updateBuilt;
bool capturedNew;
bool stop;
bool start;


// this is the the thread responsible for aligning any new frames to the global cloud
void alignFrames()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredA (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledA (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedDownsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	//icp.setRANSACOutlierRejectionThreshold(0.05);
	//icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.00001);
	icp.setMaximumIterations(50);
	boost::mutex::scoped_lock initializeLock(updateModelMutex);
	updateBuilt = true;
	*global += *cloudA;
	alignment.downsample(cloudA, lastAligned);
	initializeLock.unlock();
	cv::Mat imgA(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
	cv::Mat depthA(cloudA->height - 50, cloudA->width - 50, CV_32F );
	cv::Mat imgB(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
	cv::Mat depthB(cloudA->height - 50, cloudA->width - 50, CV_32F );

	pcl::PassThrough<pcl::PointXYZRGB> pass; 
	pass.setInputCloud( cloudA );
	pass.filter( *coloredA );
	alignment.cloudToMat(cloudA, &imgA, &depthA);
	int times = 1;
	bool first = true;
	while(!stop)
	{
		boost::mutex::scoped_lock updateCloudBLock(updateCloudBMutex);
		while(!capturedNew)
			condQ.wait( updateCloudBLock );

		// Clear old point clouds
		coloredB->clear();
		downsampledB->clear();
		transformedDownsampled->clear();
		cout << "started" << endl;
		times++;

		// Remove the NaN values from the point cloud
		pass.setInputCloud( cloudB );
		pass.filter( *coloredB );

		alignment.cloudToMat(cloudB, &imgB, &depthB);
		Transformation inverse(false);

		// use the two obtained frames to get the initial tansformations
		bool align  = alignment.getInitialTransformation(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB);

		// align is false if the two point clouds are same, so just continue and discard cloudB
		// align can also be false if the two points clouds are far so tracking has failed, in that
		// case discard the frame and wait till the user gets back to a frame that can be tracked
		if(align )
		{
			cout << "getting ICP \n";
			time_t before;
			before = time (NULL);
			Eigen::Matrix4f initialTransformation;
			inverse.get4X4Matrix(&initialTransformation);

			// Perform ICP on the downsampled point clouds
			alignment.downsample(coloredB, downsampledB);
			icp.setInputCloud(downsampledB);
			icp.setInputTarget(lastAligned);
				
			icp.align(*transformedDownsampled, initialTransformation);
			lastAligned->clear();
			pcl::copyPointCloud(*downsampledB, *lastAligned);
			std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
			time_t after = time (NULL);
			cout << "Found Transformation " << after - before << endl;
			boost::mutex::scoped_lock updateLock(updateModelMutex);
			updateBuilt = true;
			globalTransformation = globalTransformation * icp.getFinalTransformation();
			transformed->clear();
			pcl::transformPointCloud(*coloredB, *transformed, globalTransformation);
			//surfaceConst.refineSurface(transformed);
			//compressPointCloud(transformed, result);
			*global += *transformed;
			updateLock.unlock();
			cloudA = cloudB;
			coloredA = coloredB;
			imgB.copyTo(imgA);
			depthB.copyTo(depthA);
		}
		else cout << "Frame is too close ... ignoring" << endl;
		capturedNew = false;
	}
	
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{

	if (event.getKeySym () == "r" && event.keyDown ())
	{
		boost::mutex::scoped_lock startLock(updateOnlineMutex);
		std::cout << "r was pressed => starting construction " << std::endl;
		PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *onlineView ) );
		if(noFrames == 0)
		{
			cloudA = deep_copy;
			boost::thread workerThread(alignFrames);
			cout << "Started recording --> " << endl;
			start = true;
		}
		noFrames++;
		startLock.unlock();

	}else if (event.getKeySym () == "p" && event.keyDown ())
	{
		stop = true;
		condQ.notify_one();
		//surfaceConst.fastTranguilation(global);
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
		updateOnline = true;
		updateOnlineLock.unlock();
		boost::mutex::scoped_lock lock( updateCloudBMutex );
		if(seconds % 5 == 0 && start && !stop)
		{
			PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *onlineView ) );
			cloudB = deep_copy;
			capturedNew = true;
			noFrames++;
			condQ.notify_one();
		}
	}

	void run ()
	{
		pcl::Grabber* x = new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz, pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
		//pcl::Grabber* x = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
		boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		x->registerCallback (f);

		x->start ();

		while(true) sleep(1);

		x->stop ();
	}

};

void visualize()  
{  
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Kinstruct"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred);

	viewer->setBackgroundColor (0.3, 0.3, 0.3);
	viewer->addText("Result in RGB", 10, 10, "text");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbColored(global);

	
	viewer->addCoordinateSystem (1.0);

	int i = 0;
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::mutex::scoped_lock updateBuiltLock(updateModelMutex);

		if(updateBuilt)
		{
			if (!viewer->updatePointCloud<pcl::PointXYZRGB>(global, rgbColored, "result")) 
			{
				viewer->addPointCloud<pcl::PointXYZRGB> (global, rgbColored, "result");
				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result");
			}
			updateBuilt = false;
		}
		updateBuiltLock.unlock();
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
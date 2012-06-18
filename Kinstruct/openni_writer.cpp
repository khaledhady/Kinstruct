#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <pcl/registration/lum.h>
#include <pcl/registration/elch.h>
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
int loopThreshold = 0;
bool changeMsg = false;
string resolution;
string trackingMethod;
int trackingThreshold;
string state;
Alignment alignment;
SurfaceConstruction surfaceConst;
std::vector<cv::Mat *> keyframes;


// Global cloud that will hold the reconstructed model
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global (new pcl::PointCloud<pcl::PointXYZRGB>);


pcl::PointCloud<pcl::PointXYZ>::Ptr graph (new pcl::PointCloud<pcl::PointXYZ>);


pcl::registration::ELCH<pcl::PointXYZRGB> *elch = new pcl::registration::ELCH<pcl::PointXYZRGB>();
pcl::PointXYZ kinectPos(0, 0, 0);
pcl::PointXYZ frameCenter(0, 0, 0);

// Cloud holding the current feed from the Kinect camera
pcl::PointCloud<pcl::PointXYZRGB>::Ptr onlineView (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);

// Clouds holding the two views to be aligned
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);

// This holds that last downsampled cloud aligned
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastAligned (new pcl::PointCloud<pcl::PointXYZRGB>);

// The global transformation
Eigen::Matrix4f globalTransformation = Eigen::Matrix4f::Identity ();
Eigen::Affine3f kinectPosTransformation = Eigen::Affine3f::Identity();
// to hold the constructed mesh
pcl::PolygonMesh triangles;

// Mutex to manage access to the constructed model
boost::mutex updateModelMutex;

// Mutex to manage access to the online view
boost::mutex updateOnlineMutex;

// Mutext to manage any read or write to the cloud that will be used for construction
boost::mutex updateCloudBMutex;
boost::condition_variable condQ;

boost::mutex optimizingMutex;
bool optimizing;
// indicating any events
bool updateOnline;
bool updateBuilt;
bool capturedNew;
bool stop;
bool start;

ofstream myfile;

  

void initialize()
{
	noFrames = 0;
	stop = false;
	start = false;
	updateOnline = false;
	updateBuilt = false;
	optimizing = false;
	globalTransformation = Eigen::Matrix4f::Identity ();
	kinectPosTransformation = Eigen::Affine3f::Identity();	
	global->clear();
	graph->clear();
	keyframes.clear();
	elch = new pcl::registration::ELCH<pcl::PointXYZRGB>();
}

int checkLoop(int no)
{
		
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (graph);


  // K nearest neighbor search

  int K = 5;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "Frame number = " << no << endl;

  if ( kdtree.radiusSearch(kinectPos, 0.15, pointIdxNKNSearch, pointNKNSquaredDistance, 5) > 0 )
  {
	  double min = 0.0225;
	  double minIndex = -1;
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	{
		if(pointIdxNKNSearch[i] < no - loopThreshold && pointNKNSquaredDistance[i] <= min)
		{
			min = pointNKNSquaredDistance[0];
			minIndex = pointIdxNKNSearch[i];
		}
		
	}
	if (minIndex != -1)
	{
		Tracker tracker;
	
		/*tracker.setConfidenceLevel(0.98);
		tracker.setMinDistanceToEpipolar(1.0);
		tracker.setRatio(0.65f);*/
		cv::Ptr<cv::FeatureDetector> pfd=
		new cv::SurfFeatureDetector(10);
		//Estimating Projective Relations in Images
		//240
		//rmatcher.setFeatureDetector(pfd);
		//// Match the two images
		cout << keyframes.size() << "   ----- " << endl;
		std::vector<cv::DMatch> matches;
		std::vector<cv::KeyPoint> keypoints1, keypoints2;
		tracker.match(*keyframes.at(keyframes.size() -1),*keyframes.at(minIndex),
		matches, keypoints1, keypoints2);
		
	cout << "index of match " << minIndex << endl;
		cout << "found matches " <<  matches.size() << endl;
		if (matches.size() < 10)
		{
			minIndex = -1;
			cout << "Not enough visual matches" << endl;
		}
		else
		{
			cv::Mat imageMatches;
			cv::drawMatches(
			*keyframes.at(keyframes.size() -1),keypoints1, // 1st image and its keypoints
			*keyframes.at(minIndex),keypoints2, // 2nd image and its keypoints
			matches, // the matches
			imageMatches, // the image produced
			cv::Scalar(255,255,255)); // color of the lines
			cv::imshow( "Matches " , imageMatches);
			cv::waitKey(30);
		}

	}
	//std::cout << "index to match with  = " << minIndex << endl;
	return minIndex;
  }
	return -1;
}

void globalOptimization()
{

	cout << "started thread" << endl;
	elch->compute();
	cout << "Computed" << endl;
	boost::mutex::scoped_lock updateLock(updateModelMutex);
	global->clear();
	pcl::registration::ELCH<pcl::PointXYZRGB>::LoopGraphPtr mygraph = elch->getLoopGraph();
	for (size_t i = 0; i < num_vertices (*mygraph); i++)
	{
   
		*global += *(*mygraph)[i].cloud;
	}
	//*global += *temp;
	updateBuilt = true;
	optimizing = false;
	cout << "finished optimization" << endl;
}

// this is the the thread responsible for aligning any new frames to the global cloud
void alignFrames()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredA (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledA (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedDownsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	//icp.setRANSACOutlierRejectionThreshold(0.05);
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.00001);
	icp.setMaximumIterations(30);
	boost::mutex::scoped_lock initializeLock(updateModelMutex);
	updateBuilt = true;
	*global += *cloudA;

	alignment.downsample(cloudA, lastAligned);
	initializeLock.unlock();
	cv::Mat imgA(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
	cv::Mat depthA(cloudA->height - 50, cloudA->width - 50, CV_32F );
	

	pcl::PassThrough<pcl::PointXYZRGB> pass; 
	pass.setInputCloud( cloudA );
	pass.filter( *coloredA );

	alignment.cloudToMat(cloudA, &imgA, &depthA);
	keyframes.push_back(new cv::Mat(imgA));
	graph->points.push_back(kinectPos);
	elch->addPointCloud(coloredA);
	elch->addPointCloud(coloredA);
	 
	 
	//pcl::io::savePCDFileBinary("0.pcd", *cloudA);
	 
	int no = 1;
	int framesBeforeCheck = loopThreshold;
	long totalTime = 0;
	myfile.open ("results.txt");
	while(!stop)
	{
		boost::mutex::scoped_lock updateCloudBLock(updateCloudBMutex);
		while(!capturedNew && !stop)
			condQ.wait( updateCloudBLock );

		// Clear old point clouds
		coloredB->clear();
		downsampledB->clear();
		transformedDownsampled->clear();
		cv::Mat imgB(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
		cv::Mat depthB(cloudA->height - 50, cloudA->width - 50, CV_32F );
		//cout << "started" << endl;
		

		// Remove the NaN values from the point cloud
		pass.setInputCloud( cloudB );
		pass.filter( *coloredB );

		alignment.cloudToMat(cloudB, &imgB, &depthB);


		Transformation inverse(false);
		time_t beforeTracking;
		beforeTracking = time (NULL);
		
		bool align = false;
		// use the two obtained frames to get the initial tansformations
		if(trackingMethod.compare("FLOW") == 0)
			align  = alignment.getInitialTransformation(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB, trackingThreshold);
		else align  = alignment.getInitialTransformationSURF(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB, trackingThreshold);
		//bool align  = alignment.getInitialTransformationSURF(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB);
		time_t afterTracking = time (NULL);
		
		
		// align is false if the two point clouds are same, so just continue and discard cloudB
		// align can also be false if the two points clouds are far so tracking has failed, in that
		// case discard the frame and wait till the user gets back to a frame that can be tracked
		if(align )
		{
			//stringstream noString;
			//noString << no << ".pcd";
			//pcl::io::savePCDFileBinary(noString.str(), *cloudB);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
			//cout << "getting ICP \n";
			time_t before;
			before = time (NULL);
			Eigen::Matrix4f initialTransformation;
			inverse.get4X4Matrix(&initialTransformation);

			// Perform ICP on the downsampled point clouds
			alignment.downsample(coloredB, downsampledB);
			icp.setInputCloud(downsampledB);
			icp.setInputTarget(lastAligned);
				
			icp.align(*transformedDownsampled, initialTransformation);
			time_t after = time (NULL);
			lastAligned->clear();
			pcl::copyPointCloud(*downsampledB, *lastAligned);
			//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
			totalTime += after - before;
			myfile << "Found Transformation " << after - before << endl;
			myfile << "Found initial " << afterTracking - beforeTracking << endl;
			/*cout << "Found Transformation " << after - before << endl;
			cout << "Found initial " << afterTracking - beforeTracking << endl;*/
			totalTime += afterTracking - beforeTracking;
			boost::mutex::scoped_lock updateLock(updateModelMutex);
			updateBuilt = true;
			globalTransformation = globalTransformation * icp.getFinalTransformation();
			pcl::transformPointCloud(*coloredB, *transformed, globalTransformation);
			elch->addPointCloud(transformed);
			Eigen::Matrix3f rot;
			Eigen::Vector3f trans;
			pcl::PointXYZ origin(0, 0, 0);
			rot = globalTransformation.block<3, 3> (0, 0);
			trans = globalTransformation.block<3, 1> (0, 3);
			kinectPos.getVector3fMap () = rot * origin.getVector3fMap () + trans;
			graph->points.push_back(kinectPos);
			*global += *transformed;
			//*temp += *transformed;
			updateLock.unlock();
			keyframes.push_back(new cv::Mat(imgB));
			 no++;
			 int loopStart = -1;
			 if(framesBeforeCheck == 0)
			   loopStart = checkLoop(no);
			if(loopStart != -1)
			{
				framesBeforeCheck = loopThreshold;
				elch->setLoopStart(loopStart + 1);
				
				elch->setLoopEnd(no);
				cout << "Optimizing" << endl;
				boost::thread optimization(globalOptimization);
				//globalOptimization();
				cout << "After starting optimizing thread" << endl;
			}
			
			
			cloudA = cloudB;
			coloredA = coloredB;
			imgB.copyTo(imgA);
			depthB.copyTo(depthA);
			
			if(framesBeforeCheck != 0)
				framesBeforeCheck--;
		}
		//else cout << "Frame is too close ... ignoring" << endl;
		capturedNew = false;
		
		
	}
	cv::destroyWindow("Current keyframe");
	cv::destroyWindow("Online");
	//pcl::io::savePCDFileBinary("helal.pcd", *global);
	myfile << "Total time " << totalTime << endl;
	myfile.close();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{

	if (event.getKeySym () == "r" && event.keyDown ())
	{
		state = "record";
		changeMsg = true;
		boost::mutex::scoped_lock startLock(updateOnlineMutex);
		initialize();
		std::cout << "r was pressed => starting construction " << std::endl;
		PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *onlineView ) );
		if(noFrames == 0)
		{
			cloudA = deep_copy;
			boost::thread workerThread(alignFrames);
			cout << "Started recording --> " << endl;
			start = true;
			stop = false;
		}
		noFrames++;
		startLock.unlock();

	}else if (event.getKeySym () == "p" && event.keyDown ())
	{
		stop = true;
		start = false;
		state = "pause";
		changeMsg = true;
		//surfaceConst.fastTranguilation(global);
		condQ.notify_one();
		
		
	}else if(event.getKeySym() == "a" && event.keyDown())
	{
		boost::mutex::scoped_lock saveLock(updateOnlineMutex);
		std::cout << "s was pressed => saving" << std::endl;
		PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *onlineView ) );
		std::stringstream tmp;
		tmp << noFrames << ".pcd";
		pcl::io::savePCDFileBinary(tmp.str() , *deep_copy);
		noFrames++;
		saveLock.unlock();
	}else if(event.getKeySym() == "h" && event.keyDown() && stop)
	{
		surfaceConst.marchingCubes(global);
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
		pcl::Grabber* x;
		if(resolution.compare("QVGA") == 0)
			x = new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz, pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
		else x = new pcl::OpenNIGrabber();

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
	string currentMsg = "Press r to start scanning, when you are done press p";
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Kinstruct"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::PointXYZ msgPos(-3, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred);
	
	viewer->setBackgroundColor (0.3, 0.3, 0.3);
	viewer->addText("Result in RGB", 10, 10, "text");
	viewer->addText3D(currentMsg, msgPos, 0.05, 1, 1, 1, "currentMsg");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbColored(global);

	
	viewer->addCoordinateSystem (1.0);

	
	pcl::PointXYZ currentFrameCenter(0, 0, 0);
	viewer->addSphere(kinectPos, 0.05, 1, 0, 0, "pose");
	viewer->addText3D ("Kinect", kinectPos, 0.05, 0, 1, 0, "kinect");
	//graph->points.push_back(kinectPos);
	
	int i = 0;
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::mutex::scoped_lock updateBuiltLock(updateModelMutex);
		if(state.compare("record") == 0 && changeMsg)
		{
			viewer->removeText3D("currentMsg");
			//viewer->removeShape("pose");
			changeMsg = false;
		}
		else if(state.compare("pause") ==0 && changeMsg)
		{
			currentMsg = "Press r to restart or m to segment";
			viewer->addText3D(currentMsg, msgPos, 0.05, 1, 1, 1, "currentMsg");
			changeMsg = false;
		}
		
		if(updateBuilt)
		{
			
			if (!viewer->updatePointCloud<pcl::PointXYZRGB>(global, rgbColored, "result")) 
			{
				viewer->addPointCloud<pcl::PointXYZRGB> (global, rgbColored, "result");
				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result");
			}
			std::stringstream tmp;
			tmp << i;
			//viewer->resetCamera();
			tmp << i;
			viewer->removeShape("pose");
			viewer->removeText3D("kinect");
			
			viewer->addSphere(kinectPos, 0.05, 1, 0, 0, "pose");
			viewer->addText3D ("Kinect", kinectPos, 0.05, 0, 1, 0, "kinect");

			updateBuilt = false;
			i++;
		}
		updateBuiltLock.unlock();
	}   
    std::cout << "Worker: finished" << std::endl; 
}  

int main ()
{
	ifstream myfile ("params.txt");
	
	if (myfile.is_open())
	{
		string trackingThr;
		string loopThr;
		getline (myfile,resolution);
		getline (myfile,trackingThr);
		getline (myfile,trackingMethod);
		getline (myfile,loopThr);
		trackingThreshold = atoi(trackingThr.c_str());
		loopThreshold = atoi(loopThr.c_str());
	    cout << resolution << endl;
		cout << trackingThreshold << endl;
		cout << trackingMethod << endl;
		cout << loopThreshold << endl;
    }
    myfile.close();
	boost::thread workerThread(visualize); 
	SimpleOpenNIViewer v;
	v.run ();
    workerThread.join();  
	return 0;
}
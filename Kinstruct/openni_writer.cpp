#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <pcl/registration/lum.h>

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

std::vector<cv::Mat> keyframes;

// Global cloud that will hold the reconstructed model
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointXYZ kinectPos;

// Cloud holding the current feed from the Kinect camera
pcl::PointCloud<pcl::PointXYZRGB>::Ptr onlineView (new pcl::PointCloud<pcl::PointXYZRGB>);

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

// indicating any events
bool updateOnline;
bool updateBuilt;
bool capturedNew;
bool stop;
bool start;

void hist(cv::Mat src)
{
    cv::Mat hsv;
    cv::cvtColor(src, hsv, CV_BGR2HSV);

    // let's quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    cv::MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    cv::calcHist( &hsv, 1, channels, cv::Mat(), // do not use mask
        hist, 2, histSize, ranges,
        true, // the histogram is uniform
        false );
    double maxVal=0;
    cv::minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    cv::Mat histImg = cv::Mat::zeros(sbins*scale, hbins*10, CV_8UC3);

    for( int h = 0; h < hbins; h++ )
        for( int s = 0; s < sbins; s++ )
        {
            float binVal = hist.at<float>(h, s);
            int intensity = cvRound(binVal*255/maxVal);

            cvRectangle( &histImg, cv::Point(h*scale, s*scale),
                         cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
                        cv:: Scalar::all(intensity),
                         CV_FILLED );
        }

    cv::namedWindow( "Source", 1 );
    cv::imshow( "Source", src );

   cv::namedWindow( "H-S Histogram", 1 );
    cv::imshow( "H-S Histogram", histImg );

   cv::waitKey(30);
}
bool checkLoop(cv::Mat imgA, int no)
{
	Tracker tracker;
	for(int i = 0; i < keyframes.size() - 1; i++)
	{
		if(no != i)
		{
			hist(imgA);
			hist(keyframes[i]);
			/*cv::Mat imageA, imageB;
			imgA.copyTo(imageA);
			keyframes[i].copyTo(imageB);
			int tracked = tracker.track(imageA, imageB);
			cout << "got tracked " << tracked << endl;
			if(tracked > 70)
			{
					cout << "LOOP ?? " << endl;
					return true;
			}*/
		}
	}
	return false;
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
	//icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.00001);
	icp.setMaximumIterations(100);
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

	// pcl::registration::LUM<pcl::PointXYZRGB> lum;
	//lum.addPointCloud(cloudA);
	int no = 0;
	//keyframes.push_back(imgA);
	while(!stop)
	{
		boost::mutex::scoped_lock updateCloudBLock(updateCloudBMutex);
		while(!capturedNew)
			condQ.wait( updateCloudBLock );

		// Clear old point clouds
		coloredB->clear();
		downsampledB->clear();
		transformedDownsampled->clear();
		//cout << "started" << endl;
		

		// Remove the NaN values from the point cloud
		pass.setInputCloud( cloudB );
		pass.filter( *coloredB );

		alignment.cloudToMat(cloudB, &imgB, &depthB);

		//keyframes.push_back(imgB);

		/*if(checkLoop(imgA, no))
			break;*/

		Transformation inverse(false);

		// use the two obtained frames to get the initial tansformations
		bool align  = alignment.getInitialTransformation(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB);

		// align is false if the two point clouds are same, so just continue and discard cloudB
		// align can also be false if the two points clouds are far so tracking has failed, in that
		// case discard the frame and wait till the user gets back to a frame that can be tracked
		if(align )
		{
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
			lastAligned->clear();
			pcl::copyPointCloud(*downsampledB, *lastAligned);
			//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
			time_t after = time (NULL);
			//cout << "Found Transformation " << after - before << endl;
			boost::mutex::scoped_lock updateLock(updateModelMutex);
			updateBuilt = true;
			globalTransformation = globalTransformation * icp.getFinalTransformation();
			pcl::transformPointCloud(*coloredB, *transformed, globalTransformation);
			
			
			//lum.addPointCloud (transformed);
			
			
			
			//surfaceConst.compressPointCloud(transformed, result);
			*global += *transformed;
			updateLock.unlock();
			cloudA = cloudB;
			coloredA = coloredB;
			imgB.copyTo(imgA);
			depthB.copyTo(depthA);
		}
		//else cout << "Frame is too close ... ignoring" << endl;
		capturedNew = false;
		no++;
	}

	/*surfaceConst.fastTranguilation(global);*/
 
 // Use your favorite pairwise correspondence estimation algorithm(s)
 //corrs_0_to_1 = someAlgo (cloud_0, cloud_1);
 //corrs_1_to_2 = someAlgo (cloud_1, cloud_2);
 //corrs_2_to_0 = someAlgo (lum.getPointCloud (2), lum.getPointCloud (0));
 //// Add the correspondence results as edges to the SLAM graph
 //lum.setCorrespondences (0, 1, corrs_0_to_1);
 //lum.setCorrespondences (1, 2, corrs_1_to_2);
 //lum.setCorrespondences (2, 0, corrs_2_to_0);
 //// Change the computation parameters
 //lum.setMaxIterations (10);
 //lum.setConvergenceDistance (0);
 //lum.setConvergenceAngle (0);
 // Perform the actual LUM computation
// lum.compute ();
 // Return the concatenated point cloud result
 //cloud_out = lum.getConcatenatedCloud ();
 // Return the separate point cloud transformations
 /*for(int i = 0; i < lum.getNumVertices (); i++)
 {
   transforms_out[i] = lum.getTransformation (i);
 }*/
	
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
		//surfaceConst.fastTranguilation(global);
		condQ.notify_one();
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

	Eigen::Matrix3f rot;
	Eigen::Vector3f trans;
	pcl::PointXYZ origin(0, 0, 0);

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
			std::stringstream tmp;
			tmp << i;
			viewer->resetCamera();
			
			rot = globalTransformation.block<3, 3> (0, 0);
			trans = globalTransformation.block<3, 1> (0, 3);
			kinectPos.getVector3fMap () = rot * origin.getVector3fMap () + trans;
			//viewer->addText3D("here", kinectPos, 0., 1, 1, 1, tmp.str());
			viewer->addSphere(kinectPos, 0.05, 0.5, 0.5, 0.5, tmp.str());
			updateBuilt = false;
			i++;
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
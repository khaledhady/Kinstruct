//#include "Commons.h"
//#include "Tracker.h"
//#include "HornMethod.h"
//#include "Alignment.h"
//#include "SurfaceConstruction.h"
//#include <math.h>
//#include <time.h>
//#include <boost/thread.hpp>  
//#include <boost/date_time.hpp>  
//#include "pcl/win32_macros.h"
//
//using namespace std;
//using namespace pcl;
//int window = 0;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr global (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultColored (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastAligned (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//// Clouds holding the two views to be aligned
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//// The global transformation
//Eigen::Matrix4f globalTransformation = Eigen::Matrix4f::Identity ();
//Eigen::Affine3f kinectPosTransformation = Eigen::Affine3f::Identity();
//
//pcl::PolygonMesh triangles;
//Alignment alignment;
//bool mesh = false;
//bool update;
//boost::mutex updateModelMutex;
//   
//void visualize()  
//{  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Kinstruct"));
//	viewer->setBackgroundColor (0, 0, 0);
//
//	viewer->setBackgroundColor (0.3, 0.3, 0.3);
//	viewer->addText("Result in RGB", 10, 10, "text");
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbColored(global);
//
//	
//	viewer->addCoordinateSystem (1.0);
//
//	
//	pcl::PointXYZ currentFrameCenter(0, 0, 0);
//	//viewer->addSphere(kinectPos, 0.05, 1, 0, 0, "pose");
//	//viewer->addText3D ("Kinect", kinectPos, 0.05, 0, 1, 0, "kinect");
//	//graph->points.push_back(kinectPos);
//	
//	int i = 0;
//	while (!viewer->wasStopped ())
//	{
//		viewer->spinOnce (100);
//		boost::mutex::scoped_lock updateBuiltLock(updateModelMutex);
//
//		if(update)
//		{
//			if (!viewer->updatePointCloud<pcl::PointXYZRGB>(global, rgbColored, "result")) 
//			{
//				viewer->addPointCloud<pcl::PointXYZRGB> (global, rgbColored, "result");
//				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result");
//			}
//			std::stringstream tmp;
//			tmp << i;
//			//viewer->resetCamera();
//			
//			//viewer->addLine(kinectPos, currentFrameCenter, 0, 0, 1, tmp.str());
//			tmp << i;
//			//viewer->removeShape("pose");
//			viewer->removeText3D("kinect");
//			//viewer->addSphere(kinectPos, 0.05, 1, 0, 0, tmp.str());
//			//viewer->addText3D ("Kinect", kinectPos, 0.05, 0, 1, 0, "kinect");
//
//			update = false;
//			i++;
//		}
//		updateBuiltLock.unlock();
//	}   
//    std::cout << "Worker: finished" << std::endl;  
//}  
//
//int main()
//{
//
//	int times = 1;
//	int final = 15;
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredA (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledA (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredB (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledB (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedDownsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//	icp.setMaxCorrespondenceDistance(0.1);
//	//icp.setRANSACOutlierRejectionThreshold(0.05);
//	//icp.setTransformationEpsilon(1e-6);
//	icp.setEuclideanFitnessEpsilon(0.00001);
//	icp.setMaximumIterations(100);
//	std::stringstream streamA;
//	streamA << "" << times << ".pcd";
//	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (streamA.str(), *cloudA) == -1) //* load the file
//	{
//		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//		return (-1);
//	}
//	boost::mutex::scoped_lock initializeLock(updateModelMutex);
//	update = true;
//	*global += *cloudA;
//	alignment.downsample(cloudA, lastAligned);
//	initializeLock.unlock();
//	cv::Mat imgA(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
//	cv::Mat depthA(cloudA->height - 50, cloudA->width - 50, CV_32F );
//	cv::Mat imgB(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
//	cv::Mat depthB(cloudA->height - 50, cloudA->width - 50, CV_32F );
//
//	pcl::PassThrough<pcl::PointXYZRGB> pass; 
//	pass.setInputCloud( cloudA );
//	pass.filter( *coloredA );
//	cout << cloudA->points.size();
//	alignment.cloudToMat(cloudA, &imgA, &depthA);
//
//	boost::thread workerThread(visualize);
//    double total = 0;
//	while(times < final)
//	{
//		times++;
//		// Clear old point clouds
//		coloredB->clear();
//		downsampledB->clear();
//		transformedDownsampled->clear();
//		//cout << "started" << endl;
//
//		std::stringstream streamB;
//		streamB << "" << times << ".pcd";
//		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (streamB.str(), *cloudB) == -1) //* load the file
//		{
//			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//			return (-1);
//		}
//		
//
//		// Remove the NaN values from the point cloud
//		pass.setInputCloud( cloudB );
//		pass.filter( *coloredB );
//
//		alignment.cloudToMat(cloudB, &imgB, &depthB);
//
//
//		Transformation inverse(false);
//
//		// use the two obtained frames to get the initial tansformations
//		bool align  = alignment.getInitialTransformation(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB);
//
//		// align is false if the two point clouds are same, so just continue and discard cloudB
//		// align can also be false if the two points clouds are far so tracking has failed, in that
//		// case discard the frame and wait till the user gets back to a frame that can be tracked
//		if(align )
//		{
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
//			//cout << "getting ICP \n";
//			time_t before;
//			before = time (NULL);
//			Eigen::Matrix4f initialTransformation;
//			inverse.get4X4Matrix(&initialTransformation);
//
//			// Perform ICP on the downsampled point clouds
//			alignment.downsample(coloredB, downsampledB);
//			icp.setInputCloud(downsampledB);
//			icp.setInputTarget(lastAligned);
//				
//			icp.align(*transformedDownsampled, initialTransformation);
//			lastAligned->clear();
//			pcl::copyPointCloud(*downsampledB, *lastAligned);
//			//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//			time_t after = time (NULL);
//			//cout << "Found Transformation " << after - before << endl;
//			boost::mutex::scoped_lock updateLock(updateModelMutex);
//			update = true;
//			globalTransformation = globalTransformation * icp.getFinalTransformation();
//			pcl::transformPointCloud(*coloredB, *transformed, globalTransformation);
//			//elch.addPointCloud(transformed);
//			Eigen::Matrix3f rot;
//			Eigen::Vector3f trans;
//			pcl::PointXYZ origin(0, 0, 0);
//			rot = globalTransformation.block<3, 3> (0, 0);
//			trans = globalTransformation.block<3, 1> (0, 3);
//			//kinectPos.getVector3fMap () = rot * origin.getVector3fMap () + trans;
//			//graph->points.push_back(kinectPos);
//			*global += *transformed;
//			updateLock.unlock();
//			
//			cloudA = cloudB;
//			coloredA = coloredB;
//			imgB.copyTo(imgA);
//			depthB.copyTo(depthA);
//
//			//int loopStart = checkLoop(no);
//			//if(loopStart != -1)
//			//{
//			//	framesBeforeCheck = 10;
//			//	elch.setLoopStart(loopStart + 1);
//			//	
//			//	elch.setLoopEnd(no);
//			//	cout << "Optimizing" << endl;
//			//	//boost::thread optimization(globalOptimization);
//			//	globalOptimization();
//			//	cout << "After starting optimizing thread" << endl;
//			//}
//
//
//			
//			/*if(framesBeforeCheck != 0)
//				framesBeforeCheck--;*/
//		}
//	}
//	cout << "total Time is " << total << endl;
//	//refineSurface(global);
//  // Create the filtering object
//	
//	workerThread.join();  
//    
// 
//
//    return 0;
//}
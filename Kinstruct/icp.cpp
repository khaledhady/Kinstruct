//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <windows.h>
//#include <opencv\cv.h>
//#include "RobustMatcher.h"
//#include "HornMethod.h"
////#include <pcl/io/pcd_io.h>
////#include <pcl/point_types.h>
////#include <pcl/registration/icp.h>
//
//#define START_FRAME 100
//#define STEP 50
//#define LAST_FRAME 200
//using namespace std;
//
//#include <math.h>
//
//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//
//
//bool update;
//boost::mutex updateModelMutex;
//
//void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)  
//{  
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//	viewer->setBackgroundColor (0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->addCoordinateSystem (1.0);
//	viewer->resetCameraViewpoint("sample cloud");
//	/*pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
//	PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();*/
//
//	while (!viewer->wasStopped ())
//	{
//		viewer->spinOnce (100);
//	  
//		// output pointcloud
//		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());*/
//
//		/*boost::mutex::scoped_lock updateLock(updateModelMutex);
//		
//		if(update)
//		{
//			viewer->updatePointCloud(cloud, "sample cloud");
//			update = false;
//		}
//		updateLock.unlock();
//     */
//		// decompress point cloud
//	  
//		// PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
//		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//	}   
//    std::cout << "Worker: finished" << std::endl;  
//}  
// 
//Eigen::Matrix4f getICPTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out  )
//{
// 
//	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//  icp.setInputCloud(cloud_in);
//  icp.setInputTarget(cloud_out);
//  
//  pcl::PointCloud<pcl::PointXYZRGB> Final;
//  icp.setMaxCorrespondenceDistance(0.5);
//  icp.setRANSACOutlierRejectionThreshold(0.1);
//  icp.setTransformationEpsilon(0.001);
//  icp.align(Final);
//  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//  //std::cout << icp.getFinalTransformation() << std::endl;
//
//  return icp.getFinalTransformation();
//}
//
////PointCloud<FPFHSignature33> fpfhs;
////
//// 
////
//////want to place featues rowwise
////
////int rows = fpfhs.points.size();
////
////int cols = sizeof(FPFHSignature33)/sizeof(fpfhs.histogram[0]);
//
////cv::Mat mat_header_with_disbled_reference_counting(rows, cols, CV_32F, (void*)&fpfhs.points[0]);
//
//void cvMatToPointCloud2(cv::Mat *color, cv::Mat *depth , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
//{
//
//
//	int rowsRGB = color->rows;
//	int colsRGB = color->cols;
//	int noPixels = color->rows * color->cols;
//
//	cloud_out->width    = colsRGB;
//	cloud_out->height   = rowsRGB;
//	cloud_out->is_dense = false;
//	cloud_out->points.resize (noPixels);
//	
//	int cloudIndex = 0 ; 
//	
//	for (int k = 0; k < rowsRGB ; k++) {
//		for (int m = 0; m < colsRGB; m++) {
//			pcl::PointXYZRGB point;
//			int bit0 = depth->at<cv::Vec3b>(k,m)[0];
//			int bit1 = depth->at<cv::Vec3b>(k,m)[1];
//			int dep = (bit0 | bit1 << 8 );
//				
//			double blue = color->at<cv::Vec3b>(k,m)[0];
//			double green = color->at<cv::Vec3b>(k,m)[1];
//			double red = color->at<cv::Vec3b>(k,m)[2];
//			
//			
//				blue = 0;
//				red = 0;
//				green = 255;
//			
//
//			point.b = blue ; 
//			point.g = green;
//			point.r = red ; 
//			point.x = m ;
//			point.y = -k ;
//			point.z = dep / 100 ; 
//				
//			cloud_out->points[cloudIndex] = point ; 
//			cloudIndex ++ ;
//
//			if(cloudIndex == 107 )
//			{
//				int y = 0  ;
//			}
//			
//		}
//
//		//std::cout<< "index now = " << cloudIndex << std::endl ; 
//	}
//
//	int x = 0 ;
//	
//	/*delete vertices;
//	delete colors;*/
//}
//void cvMatToPointCloud(cv::Mat *color, cv::Mat *depth , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
//{
//
//
//	int rowsRGB = color->rows;
//	int colsRGB = color->cols;
//	int noPixels = color->rows * color->cols;
//
//	cloud_out->width    = colsRGB;
//	cloud_out->height   = rowsRGB;
//	cloud_out->is_dense = false;
//	cloud_out->points.resize (noPixels);
//	
//	int cloudIndex = 0 ; 
//	
//	for (int k = 0; k < rowsRGB ; k++) {
//		for (int m = 0; m < colsRGB; m++) {
//			pcl::PointXYZRGB point;
//			int bit0 = depth->at<cv::Vec3b>(k,m)[0];
//			int bit1 = depth->at<cv::Vec3b>(k,m)[1];
//			int dep = (bit0 | bit1 << 8 );
//				
//			double blue = color->at<cv::Vec3b>(k,m)[0];
//			double green = color->at<cv::Vec3b>(k,m)[1];
//			double red = color->at<cv::Vec3b>(k,m)[2];
//			
//				/*blue = 0;
//				red = 255;
//				green = 0;*/
//			
//			point.b = blue ; 
//			point.g = green;
//			point.r = red ; 
//			point.x = m ;
//			point.y = -k ;
//			point.z = dep / 100 ; 
//				
//			cloud_out->points[cloudIndex] = point ; 
//			cloudIndex ++ ;
//
//			if(cloudIndex == 107 )
//			{
//				int y = 0  ;
//			}
//			
//		}
//
//		//std::cout<< "index now = " << cloudIndex << std::endl ; 
//	}
//
//	int x = 0 ;
//	
//	/*delete vertices;
//	delete colors;*/
//}
//
//void vector3DToPointCloud(cv::Mat* color , vector<cv::Point3f *> setA , pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
//{
//
//
//	
//	int noPixels = setA.size();
//
//	cloud_out->width    = noPixels;
//	cloud_out->height   = 1 ;
//	cloud_out->is_dense = false;
//	cloud_out->points.resize (noPixels);
//
//	int cloudIndex = 0 ; 
//	
//	for (int k = 0; k < noPixels ; k++) {
//		
//			pcl::PointXYZRGB point;
//			float x = setA[k]->x ;
//			float y = setA[k]->y ;
//			float z = setA[k]->z ;
//
//
//			point.x = x ;
//			point.y = y ;
//			point.z = z ; 
//				
//
//			double blue = color->at<cv::Vec3b>(y,x)[0];
//			double green = color->at<cv::Vec3b>(y,x)[1];
//			double red = color->at<cv::Vec3b>(y,x)[2];
//
//			point.r = red ; 
//			point.g = green ; 
//			point.b = blue ; 
//			cloud_out->points[cloudIndex] = point ; 
//			cloudIndex ++ ;
//
//			if(cloudIndex == 107 )
//			{
//				int y = 0  ;
//			}
//			
//		}
//
//}
//
//Eigen::Matrix4f alignFrames(cv::Mat *colorA, cv::Mat *colorB, cv::Mat *depthA, cv::Mat *depthB)
//{
//	
//	cv::Mat grayA, grayB;
//	cvtColor(*colorA, grayA, CV_RGB2GRAY);
//	cvtColor(*colorB, grayB, CV_RGB2GRAY);
//
//	RobustMatcher rmatcher;
//
//	cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector(10);
//
//	rmatcher.setFeatureDetector(pfd);
//	// Match the two images
//	vector<cv::DMatch> matches;
//    vector<cv::KeyPoint> keypointsA, keypointsB;
//	cv::Mat fundemental= rmatcher.match(grayA, grayB, matches, keypointsA, keypointsB);
//	
//
//	cv::Mat img_matches;
//	drawMatches( grayA, keypointsA, grayB, keypointsB,
//               matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
//               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//  
//    //-- Show detected matches
//    imshow( "Good Matches", img_matches );
//
//	vector<cv::Point2f> selPoints1, selPoints2;
//	cv::KeyPoint::convert(keypointsA, selPoints1);
//	cv::KeyPoint::convert(keypointsB, selPoints2);
//
//	vector<cv::Point3f *> setA, setB;
//	for(int i = 0; i < matches.size(); i++)
//	{
//		cv::Point2f point2D = selPoints1.at(matches.at(i).queryIdx);
//		int bit0 = depthA->at<cv::Vec3b>(point2D.y, point2D.x)[0];
//		int bit1 = depthA->at<cv::Vec3b>(point2D.y, point2D.x)[1];
//		double depth = (bit0 | bit1 << 8 );
//		cv::Point3f *point3D = new cv::Point3f(point2D.x, point2D.y, depth / 100); 
//		setA.push_back(point3D);
//
//		point2D = selPoints2.at(matches.at(i).trainIdx);
//		bit0 = depthB->at<cv::Vec3b>(point2D.y, point2D.x)[0];
//		bit1 = depthB->at<cv::Vec3b>(point2D.y, point2D.x)[1];
//		depth = (bit0 | bit1 << 8 );
//		cv::Point3f *point3DB = new cv::Point3f(point2D.x, point2D.y, depth / 100); 
//		setB.push_back(point3DB);
//	}
//	
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//	// here we want to convert setA and setB to PointCloud in and out
//
//	vector3DToPointCloud(colorA,setA , cloud_in ) ;
//	vector3DToPointCloud(colorB , setB , cloud_out ) ;
//	Eigen::Matrix4f transfromationMAt = getICPTransformation(cloud_in , cloud_out  );
//
//	return transfromationMAt ; 
//
///*
//	Transformation result(false);
//	HornMethod hornMethod;
//	hornMethod.getTransformation(setA, setB, &result);
//	inverse->invert(&result);*/
//	// clean vectors SetA and SetB
//	
//}
//
//
//int main (int argc, char** argv)
//{
//
//
//	CvCapture* captureDepth = cvCaptureFromAVI("depth.avi");
//	CvCapture* captureColor = cvCaptureFromAVI("color.avi");
//		for(int i = 0; i < 100; i++ )
//	{
//		cvQueryFrame(captureColor);
//		cvQueryFrame(captureDepth);
//	}
//
//	IplImage* img = 0; 
//	img = cvQueryFrame(captureColor);	//retrieve the captured frame
//	cv::Mat color1(img,true);
//	img = cvQueryFrame(captureDepth);	//retrieve the captured frame
//	cv::Mat depth1(img,true);
//
//
//
//  /*IplImage* img = 0;
//  img  = cvLoadImage("frame1.bmp",CV_LOAD_IMAGE_GRAYSCALE);
//  cv::Mat color1 = img ; 
//
//  img = cvLoadImage("dframe1.bmp",CV_LOAD_IMAGE_GRAYSCALE);
//  cv::Mat depth1 = img ;*/ 
//
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
//  cvMatToPointCloud(&color1 , &depth1  , cloud_in ) ; 
//
//  		for(int i =	0 ; i < 10; i++ )
//	{
//		cvQueryFrame(captureColor);
//		cvQueryFrame(captureDepth);
//	}
//  	img = cvQueryFrame(captureColor);	//retrieve the captured frame
//	cv::Mat color2(img,true);
//	IplImage*  img2 = cvQueryFrame(captureDepth);	//retrieve the captured frame
//	cv::Mat depth2(img2,true);
//
//	cv::imshow("first", color1);
//	cv::imshow("second", color2);
//	
//
//  /*img  = cvLoadImage("frame2.bmp",CV_LOAD_IMAGE_GRAYSCALE);
//  cv::Mat color2 = img ; 
//
//  img = cvLoadImage("dframe2.bmp",CV_LOAD_IMAGE_GRAYSCALE);
//  cv::Mat depth2 = img ; */
// 
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
//  cvMatToPointCloud(&color2 , &depth2  , cloud_out ) ; 
//
//
//  //// Fill in the CloudIn data
//  //cloud_in->width    = 5;
//  //cloud_in->height   = 1;
//  //cloud_in->is_dense = false;
//  //cloud_in->points.resize (cloud_in->width * cloud_in->height);
//  //for (size_t i = 0; i < cloud_in->points.size (); ++i)
//  //{
//  //  cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//  //  cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//  //  cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//  //}
//  //std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
//    //  << std::endl;
//  /*for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
//      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
//      cloud_in->points[i].z << std::endl;*/
//  //*cloud_out = *cloud_in;
//  //std::cout << "size:" << cloud_out->points.size() << std::endl;
// /* for (size_t i = 0; i < cloud_in->points.size (); ++i)
//    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;*/
// // std::cout << "Transformed " << cloud_in->points.size () << " data points:"
//   //   << std::endl;
//  /*for (size_t i = 0; i < cloud_out->points.size (); ++i)
//    std::cout << "    " << cloud_out->points[i].x << " " <<
//      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;*/
//  /*pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//  icp.setInputCloud(cloud_in);
//  icp.setInputTarget(cloud_out);
//  pcl::PointCloud<pcl::PointXYZ> Final;
//  icp.align(Final);
//  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//  icp.getFitnessScore() << std::endl;
//  std::cout << icp.getFinalTransformation() << std::endl;*/
//
//  //=====================================================================
//
//    /*pcl::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> sac;
//	sac.setInputCloud(cloud_in);
//	sac.setTargetCloud(cloud_out);
//	sac.setInlierThreshold(epsilon);
//	sac.setMaxIterations(N);
//	sac.setInputCorrespondences(correspondences);
//	sac.getCorrespondences(inliers);
//	Eigen::Matrix4f transformation = sac.
//	getBestTransformation();*/
//
//
//  //======================================================================
///*
//	pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::PointXYZRGB> sac_ia;
//	sac_ia.setNumberOfSamples (n)
//	sac_ia.setMinSampleDistance (d);
//	sac_ia.setCorrespondenceRandomness (k);
//	sac_ia.setMaximumIterations (N);
//	sac_ia.setInputCloud (source);
//	sac_ia.setInputTarget (target);
//	sac_ia.setSourceFeatures (source_features);
//	sac_ia.setTargetFeatures (target_features);
//	sac_ia.align (aligned_source);
//	Eigen::Matrix4f = sac_ia.getFinalTransformation ();*/
//
//  //======================================================================
//
//  Eigen::Matrix4f tranfromationMAt = alignFrames(&color1 , &color2 , &depth1 , &depth2 )  ;
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//  pcl::transformPointCloud(*cloud_out , *transformed , tranfromationMAt );
//
//  *cloud_in = *cloud_in +  *transformed ; 
//
//
// std::cout << tranfromationMAt<< std::endl;
//
//
//  //=========================================================================
//   /*Eigen::Matrix4f tranfromationMAt2 ; 
//  pcl::registration::TransformationEstimationSVD< pcl::PointXYZRGB ,  pcl::PointXYZRGB > SVNTransformer ; 
//  SVNTransformer.estimateRigidTransformation( *cloud_in , *cloud_out , tranfromationMAt2);
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//  pcl::transformPointCloud(*cloud_out , *transformed , tranfromationMAt2 );
//
//  *cloud_in = *cloud_in +  *transformed ; 
//
//  std::cout << tranfromationMAt2 << std::endl;*/
//
//
//
//  visualize(cloud_in);
//
//  getchar();
// return (0);
//}
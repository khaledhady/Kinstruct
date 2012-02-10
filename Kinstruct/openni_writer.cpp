//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//
//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <boost/thread.hpp>
//#include <boost/date_time.hpp>
//
//
//#ifdef WIN32
//# define sleep(x) Sleep((x)*1000) 
//#endif
//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr globalcloud;
//boost::mutex saveCloud;
//int noFrames = 0;
//#include "Commons.h"
//#include "RobustMatcher.h"
//#include "HornMethod.h"
//#include <math.h>
//#include <time.h>
//#include <boost/thread.hpp>  
//#include <boost/date_time.hpp>  
//#include "pcl/win32_macros.h"
//#define START_FRAME 200
//#define STEP 10
//#define LAST_FRAME 300
////using namespace cv;
//using namespace std;
//using namespace pcl;
//int window = 0;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultColored (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastAligned (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//		
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr one (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr two (new pcl::PointCloud<pcl::PointXYZRGB>);
//std::vector<Transformation *> transformations;
//std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> previous;
//Transformation *global = new Transformation(true);
//std::stringstream compressedData;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
//bool update;
//boost::mutex updateModelMutex;
//
//void cloudToMat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat *color, cv::Mat *depth)
//{
//	for(int i = 0; i < color->cols; i++)
//	{
//		
//		for(int j = 0; j < color->rows; j++)
//		{
//			pcl::PointXYZRGB point = cloud->at(i,j);
//			   if (!isFinite (point))
//			        continue;
//			
//			color->at<cv::Vec3b>(j, i)[0] = point.b;
//			color->at<cv::Vec3b>(j, i)[1] = point.g;
//			color->at<cv::Vec3b>(j, i)[2] = point.r;
//			depth->at<float>(j, i) = point.z;
//			
//		}
//
//	}
//	cv::imshow("here", *color);
//}
//void alignFrames(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB, 
//	             cv::Mat *colorA, cv::Mat *colorB, cv::Mat *depthA, cv::Mat *depthB, Transformation *inverse)
//{
//	// Prepare the matcher
//	cv::Mat grayA, grayB;
//	cv::cvtColor(*colorA, grayA, CV_RGB2GRAY);
//	cv::cvtColor(*colorB, grayB, CV_RGB2GRAY);
//
//	RobustMatcher rmatcher;
//
//	cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector(10);
//
//	rmatcher.setFeatureDetector(pfd);
//	// Match the two images
//	vector<cv::DMatch> matches;
//	vector<cv::KeyPoint> keypointsA, keypointsB;
//	cout << "getting matches \n";
//	time_t before;
//    before = time (NULL);
//    
//  
//	cv::Mat fundemental= rmatcher.match(grayA, grayB, matches, keypointsA, keypointsB);
//	time_t after;
//
//	after = time (NULL);
//	cout << "Found matches " << after - before << endl;
//	
//
//	cv::Mat img_matches;
//	drawMatches( grayA, keypointsA, grayB, keypointsB,
//               matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
//               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//  
//    //-- Show detected matches
//    cv::imshow( "Good Matches" , img_matches );
//	//window++;
//
//	vector<cv::Point2f> selPoints1, selPoints2;
//	cv::KeyPoint::convert(keypointsA, selPoints1);
//	cv::KeyPoint::convert(keypointsB, selPoints2);
//
//	vector<cv::Point3f> setA, setB;
//	for(int i = 0; i < matches.size(); i++)
//	{
//		cv::Point2f point2DA = selPoints1.at(matches.at(i).queryIdx);
//		cv::Point2f point2DB = selPoints2.at(matches.at(i).trainIdx);
//		
//		pcl::PointXYZRGB pointA = cloudA->at(point2DA.x, point2DA.y);
//		pcl::PointXYZRGB pointB = cloudB->at(point2DB.x, point2DB.y);
//		  if (!isFinite (pointA) || !isFinite (pointB))
//			    continue;
//		  cout << pointA.z << endl; 
//
//		cv::Point3f point3D (pointA.x, pointA.y, pointA.z); 
//		setA.push_back(point3D);
//		cv::Point3f point3DB (pointB.x, pointB.y, pointB.z); 
//		setB.push_back(point3DB);
//	}
//
//	cout << "getting transformation \n";
//	before = time (NULL);
//	Transformation result(false);
//	HornMethod hornMethod;
//	hornMethod.getTransformation(&setA, &setB, &result);
//	inverse->invert(&result);
//	after = time (NULL);
//	cout << "Found transformation " << after - before << endl;
//	// clean vectors SetA and SetB
//	
//}
//
//void transform(Transformation *transform, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target)
//{
//	int pixelIndex = 0;
//	target->points.resize (src->width * src->height);
//	target->width = src->width;
//	target->height = src->height;
//
//	for (int k = 0; k < src->size() ; k++) {
//		
//			
//			//int bit0 = depth->at<cv::Vec3b>(k,m)[0];
//			//int bit1 = depth->at<cv::Vec3b>(k,m)[1];
//		pcl::PointXYZRGB point = src->points[k];
//			   if (!isFinite (point))
//			        continue;
//			float x = src->points[k].x;
//			float y = src->points[k].y;
//			float z = src->points[k].z;
//				
//			int blue = src->points[k].b;
//			int green = src->points[k].g;
//			int red = src->points[k].r;
//
//			cv::Point3d myPoint(x, y, z);
//			transform->applyToPoint(&myPoint);
//
//			target->points[pixelIndex].x = myPoint.x;
//			target->points[pixelIndex].y = myPoint.y;
//			target->points[pixelIndex].z = myPoint.z;
//			uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
//              static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
//			target->points[pixelIndex].rgb = *reinterpret_cast<float*>(&rgb);
//			
//			pixelIndex++;
//
//			
//	}
//
//	
//
//}
//
//void makeRed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
//{
//	for(int i = 0; i < cloudToColor->size(); i++)
//	{
//		cloudToColor->points[i].r = 255;
//		cloudToColor->points[i].g = 0;
//		cloudToColor->points[i].b = 0;
//	}
//}
//
//void makeGreen(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
//{
//	for(int i = 0; i < cloudToColor->size(); i++)
//	{
//		cloudToColor->points[i].r = 0;
//		cloudToColor->points[i].g = 255;
//		cloudToColor->points[i].b = 0;
//	}
//}
//
//void makeBlue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
//{
//	for(int i = 0; i < cloudToColor->size(); i++)
//	{
//		cloudToColor->points[i].r = 0;
//		cloudToColor->points[i].g = 0;
//		cloudToColor->points[i].b = 255;
//	}
//}
//
//void makeYellow(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
//{
//	for(int i = 0; i < cloudToColor->size(); i++)
//	{
//		cloudToColor->points[i].r = 255;
//		cloudToColor->points[i].g = 255;
//		cloudToColor->points[i].b = 0;
//	}
//}
//
//void makeWhite(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudToColor)
//{
//	for(int i = 0; i < cloudToColor->size(); i++)
//	{
//		cloudToColor->points[i].r = 255;
//		cloudToColor->points[i].g = 255;
//		cloudToColor->points[i].b = 255;
//	}
//}
//
//int main1()
//{
//	
//	cout << "read input" << endl;
//	cv::Mat imgA(cloud->height, cloud->width, CV_8UC3 );
//	cv::Mat imgB(cloud2->height, cloud2->width, CV_8UC3 );
//	cv::Mat depthA(cloud->height, cloud->width, CV_32F );
//	cv::Mat depthB(cloud2->height, cloud2->width, CV_32F );
//	pcl::PassThrough<pcl::PointXYZRGB> pass; // can do this without parameters
//pass.setInputCloud( cloud );
//pass.filter( *one );
//pass.setInputCloud( cloud2 );
//pass.filter( *two );
//	cloudToMat(cloud, &imgA, &depthA);
//	cv::waitKey(0);
//	cloudToMat(cloud2, &imgB, &depthB);
//	cv::waitKey(0);	
//		if(noFrames == 1)
//			previous.push_back(one);
//
//	    Transformation *inverse = new Transformation(false);
//		alignFrames(cloud, cloud2, &imgA, &imgB, &depthA, &depthB, inverse);
//		//cv::waitKey(0);
//		//global->concatenate(inverse);
//		//transformations.push_back(inverse);
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
//		
//		/*for(int i = transformations.size() - 1; i >= 0 ; i--)
//		{
//			transform(transformations.at(i), tmp, tmp);
//		}*/
//		Eigen::Matrix4f initialTransformation;
//		inverse->get4X4Matrix(&initialTransformation);
//		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//		icp.setInputCloud(two);
//		icp.setInputTarget(previous.at(previous.size() - 1));
//		pcl::PointCloud<pcl::PointXYZRGB> Final;
//		icp.setMaxCorrespondenceDistance(0.1);
//	    icp.setEuclideanFitnessEpsilon(0.001);
//		//icp.setTransformationEpsilon(0.01);
//	    //icp.setMaximumIterations(1);
//        icp.align(*tmp, initialTransformation);
//		//pcl::copyPointCloud(*tmp, *lastAligned);
//		previous.push_back(tmp);
//		//lastAligned = (new pcl::PointCloud<pcl::PointXYZRGB>);
//		//makeGreen(lastAligned);
//		//*cloud += *lastAligned;
//		 /*pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//   viewer.showCloud (cloud);
//   while (!viewer.wasStopped ())
//   {
//   }*/
//		
//		std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGB>);
//		//transform(global, tmp, final);
//		//global->concatenate(&icp.getFinalTransformation());
//		boost::mutex::scoped_lock updateLock(updateModelMutex);
//		update = true;
//		*result += *tmp;
//		if(noFrames == 1)
//		{
//			*result += *one;
//			//makeRed(one);
//			//makeGreen(tmp);
//			*resultColored += *one;
//			
//		}else if(noFrames == 2)
//		{
//			//makeBlue(tmp);
//		}else if(noFrames == 3)
//		{
//			//makeYellow(tmp);
//		}else if(noFrames == 4)
//		{
//			//makeWhite(tmp);
//		}
//		*resultColored += *tmp;
//		updateLock.unlock();
//		pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//		viewer.showCloud (previous.at(previous.size() - 1));
//   while (!viewer.wasStopped ())
//   {
//   }
//		//inverse->applyToFrame(&imgB, &depthB, tmp);
//		if(noFrames == 2)
//		{
//			
//			//makeWhite(tmp);
//			 //  pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
//  
// // pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
//
//    // instantiate point cloud compression for encoding and decoding
//    //PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (compressionProfile, true);
//	//PointCloudEncoder->encodePointCloud (result, compressedData);
//		//pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
//	//PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();
//	//PointCloudDecoder->decodePointCloud(compressedData, cloud_filtered);
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//	
//	
//	 int v1(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//  viewer->setBackgroundColor (0, 0, 0, v1);
//  viewer->addText("Result RGB", 10, 10, "v1 text", v1);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(result);
//  viewer->addPointCloud<pcl::PointXYZRGB> (result, rgb, "sample cloud1", v1);
//
//  int v2(0);
//  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
//  viewer->addText("Result Colored", 10, 10, "v2 text", v2);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(resultColored);
//  viewer->addPointCloud<pcl::PointXYZRGB> (resultColored, rgb1, "sample cloud2", v2);
//
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
//  viewer->addCoordinateSystem (1.0);
//  viewer->resetCameraViewpoint("sample cloud1");
//       while (!viewer->wasStopped ())
//	{
//		viewer->spinOnce (100);
//	   }
//    std::cout << "main: done" << std::endl; 
//		}
//	
//    return 0;
//}
//
//
//void visualize()  
//{  
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//	
//	
//	 int v1(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//  viewer->setBackgroundColor (0, 0, 0, v1);
//  viewer->addText("Radius: 1", 10, 10, "v1 text", v1);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(result);
//  viewer->addPointCloud<pcl::PointXYZRGB> (result, rgb, "sample cloud1", v1);
//
//  int v2(0);
//  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
//  viewer->addText("Radius: 2", 10, 10, "v2 text", v2);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(resultColored);
//  viewer->addPointCloud<pcl::PointXYZRGB> (resultColored, rgb1, "sample cloud2", v2);
//
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
//  viewer->addCoordinateSystem (1.0);
//  viewer->resetCameraViewpoint("sample cloud1");
//
//	while (!viewer->wasStopped ())
//	{
//		viewer->spinOnce (100);
//	  
//	
//		// output pointcloud
//		/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());*/
//
//		boost::mutex::scoped_lock updateLock(updateModelMutex);
//		/*if(updateLock.try_lock())*/
//		if(update)
//		{
//			viewer->updatePointCloud(result, "sample cloud");
//			update = false;
//		}
//		updateLock.unlock();
//     
//		// decompress point cloud
//	  
//		// PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
//		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//	}   
//    std::cout << "Worker: finished" << std::endl;  
//}  
//
//
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
//{
//	  
//	if (event.getKeySym () == "r" && event.keyDown ())
//	{
//		boost::mutex::scoped_lock saveLock(saveCloud);
//		std::cout << "r was pressed => removing all text" << std::endl;
//			pcl::PassThrough<pcl::PointXYZRGB> pass; // can do this without parameters
//			PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *globalcloud ) );
//		if(noFrames == 0)
//		{
//			cloud = deep_copy;
//			cout << cloud->width << endl;
//		}
//		else if(noFrames == 1){
//			cloud2 = deep_copy;
//			//main1();
//		}else{
//			cloud = cloud2;
//			cloud2 = deep_copy;
//			
//			//main1();
//		}
//		noFrames++;
//		/*char fileName[20];
//		itoa(noFrames, fileName, 10);*/
//
//		std::string fileName = ".pcd";
//		std::stringstream tmp;
//		tmp << noFrames << ".pcd";
//		
//		//pcl::io::savePCDFileASCII(fileName, *globalcloud);
//		pcl::io::savePCDFileBinary(tmp.str() , *deep_copy);
//		/*if(noFrames == 5)
//			pcl::io::savePLYFile("room.ply", *result);*/
//		saveLock.unlock();
//
//	char str[512];
//	}
//}
//
//	
//
//class SimpleOpenNIViewer
//{
//public:
//	int noFrames;
//	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {  }
//	
//
//	
//
//	void save()
//	{
//		/*char fileName[20];
//		itoa(noFrames, fileName, 10);*/
//
//		/*pcl::io::savePCDFileASCII (, cloud);
//		std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;*/
//		//pcl::io::savePCDFileASCII (fileName, *cloud);
//	}
//
//	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
//	{
//
//		boost::mutex::scoped_lock saveLock(saveCloud);
//		globalcloud = cloud;
//		if (!viewer.wasStopped())
//		{
//		 viewer.showCloud (cloud);
//		}
//		saveLock.unlock();
//		
//		
//
//	}
//
//	void run ()
//	{
//		pcl::Grabber* x = new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz, pcl::OpenNIGrabber::OpenNI_QVGA_30Hz);
//		//pcl::Grabber* x = new pcl::OpenNIGrabber();
//
//		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
//			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
//
//		x->registerCallback (f);
//
//		viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)&viewer);
//
//		//viewer.registerKeyboardCallback(&SimpleOpenNIViewer::keyboardEventOccurred);
//		x->start ();
//
//		while (!viewer.wasStopped())
//		{
//			sleep (1);
//		}
//
//		x->stop ();
//	}
//
//
//	
//
//
//
//
//	pcl::visualization::CloudViewer viewer;
//};
//
//int main ()
//{
//	//boost::thread workerThread(visualize); 
//	SimpleOpenNIViewer v;
//	v.run ();
//	//workerThread.join();  
//	return 0;
//}
//

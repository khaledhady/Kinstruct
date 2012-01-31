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
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr one (new pcl::PointCloud<pcl::PointXYZRGB>);
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr two (new pcl::PointCloud<pcl::PointXYZRGB>);
//std::stringstream compressedData;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
//bool update;
//boost::mutex updateModelMutex;
//
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
//
//void
//detectNew (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB)
//{
//  srand ((unsigned int) time (NULL));
//
//  // Octree resolution - side length of octree voxels
//  float resolution = 64.0f;
//
//  // Instantiate octree-based point cloud change detection class
//  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);
//
//
//  // Add points from cloudA to octree
//  octree.setInputCloud (cloudA);
//  octree.addPointsFromInputCloud ();
//
//  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
//  octree.switchBuffers ();
//
//  // Add points from cloudB to octree
//  octree.setInputCloud (cloudB);
//  octree.addPointsFromInputCloud ();
//
//  std::vector<int> newPointIdxVector;
//
//  // Get vector of point indices from octree voxels which did not exist in previous buffer
//  octree.getPointIndicesFromNewVoxels (newPointIdxVector);
//
//  // Output points
//  
//  int index = 0;
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB> );
//  tmp->resize(newPointIdxVector.size ());
//  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
//  for (size_t i = 0; i < newPointIdxVector.size (); ++i)
//  {
//	tmp->points[index].x = cloudB->points[newPointIdxVector[i]].x;
//	tmp->points[index].y = cloudB->points[newPointIdxVector[i]].y;
//	tmp->points[index].z = cloudB->points[newPointIdxVector[i]].z;
//	tmp->points[index].r = cloudB->points[newPointIdxVector[i]].r;
//	tmp->points[index].g = cloudB->points[newPointIdxVector[i]].g;
//	tmp->points[index].b = cloudB->points[newPointIdxVector[i]].b;
//	index++;
//  }
//  *cloudA += *tmp;
//
//}
//void alignFrames(cv::Mat *colorA, cv::Mat *colorB, cv::Mat *depthA, cv::Mat *depthB, Transformation *inverse)
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
//		pcl::PointXYZRGB pointA = cloud->at(point2DA.x, point2DA.y);
//		pcl::PointXYZRGB pointB = cloud2->at(point2DB.x, point2DB.y);
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
//   
//void visualize()  
//{  
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//	viewer->setBackgroundColor (0, 0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//	viewer->addCoordinateSystem (1.0);
//	viewer->resetCameraViewpoint("sample cloud");
//	pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
//	PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();
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
//			viewer->updatePointCloud(cloud, "sample cloud");
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
//int main()
//{
//	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("bed1.pcd", *cloud) == -1) //* load the file
//  {
//    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//    return (-1);
//  }
//
//	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("bed2.pcd", *cloud2) == -1) //* load the file
//  {
//    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//    return (-1);
//  }
//	cout << "read input" << endl;
//	cv::Mat imgA(480, 640, CV_8UC3 );
//	cv::Mat imgB(480, 640, CV_8UC3 );
//	cv::Mat depthA(480, 640, CV_32F );
//	cv::Mat depthB(480, 640, CV_32F );
//	pcl::PassThrough<pcl::PointXYZRGB> pass; // can do this without parameters
//pass.setInputCloud( cloud );
//pass.filter( *one );
//pass.setInputCloud( cloud2 );
//pass.filter( *two );
//	cloudToMat(cloud, &imgA, &depthA);
//	cv::waitKey(0);
//	cloudToMat(cloud2, &imgB, &depthB);
//	cv::waitKey(0);
//	//return 0;
////}
//
////int main1 () {
//// 
////	CvCapture* captureDepth = cvCaptureFromAVI("depth.avi");
////	CvCapture* captureColor = cvCaptureFromAVI("color.avi");
////
////	for(int i = 0; i < START_FRAME; i++ )
////	{
////		cvQueryFrame(captureColor);
////		cvQueryFrame(captureDepth);
////	}
////	
////	IplImage* img = 0; 
////	img = cvQueryFrame(captureColor);	//retrieve the captured frame
////	cv::Mat imgA(img,true);
////	img = cvQueryFrame(captureDepth);	//retrieve the captured frame
////	cv::Mat depthA(img,true);
////
////
////	int noPixels = imgA.rows * imgA.cols;
////	
////	
////  /*pcl::io::savePCDFileASCII ("test.pcd", cloud);
////  cout << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;*/
////
////  
////
//	int frameNo = START_FRAME;
//	//Transformation *global = new Transformation(true);
//	//global->applyToFrame(&imgA, &depthA, cloud_filtered);
////	//boost::thread workerThread(visualize); 
////	
////   pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
////  
////  pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR;
////
////    // instantiate point cloud compression for encoding and decoding
////    PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (compressionProfile, false);
////	// stringstream to store compressed point cloud
////      /*std::stringstream compressedData;*/
//	int times = 1;
////	pcl::PointCloud<pcl::PointXYZRGB>::Ptr collect (new pcl::PointCloud<pcl::PointXYZRGB>);
////	while(frameNo <= LAST_FRAME)
////	{
////		
////		for(int j = 0; j < STEP; j++)
////		{
////			cvQueryFrame(captureColor);
////			cvQueryFrame(captureDepth);
////		}
////
////		img = cvQueryFrame(captureColor);	//retrieve the captured frame
////		cv::Mat imgB(img, true);
////		img = cvQueryFrame(captureDepth);
////		cv::Mat depthB(img, true);
////
//	    Transformation *inverse = new Transformation(false);
//		alignFrames(&imgA, &imgB, &depthA, &depthB, inverse);
//		//global->concatenate(inverse);
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
//		transform(inverse, two, tmp);
//		//inverse->applyToFrame(&imgB, &depthB, tmp);
//		boost::mutex::scoped_lock updateLock(updateModelMutex); // defer_lock makes it initially unlocked
//			update = true;
//			/*detectNew(cloud, tmp);*/
//			//*collect += *tmp;
//			*cloud += *tmp;
//			//*cloud_filtered += *cloud;
//
//
//
//
//  // Create the filtering object
//			
//
//
//		updateLock.unlock();
//		
//		// compress point cloud
//		if(times % 5 == 0)
//         {
//			 //detectNew(cloud, collect);
//			 //PointCloudEncoder->encodePointCloud (cloud, compressedData);
//			 //collect->clear();
//			cout << "Compressed" << endl;
//		}
//		imgB.copyTo(imgA);
//		depthB.copyTo(depthA);
//		frameNo += STEP;
//		times++;
//		//viewer->spinOnce(1000);
//		//break;
//	//}
//	
//	/*viewer->spinOnce(100);
//
//	}*/
//    
//	 /*pcl::io::savePCDFileASCII ("test.pcd", *cloud);
//  cout << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;*/
//	//Visualizer::start(argc, argv);
//
//	
//  
//	//pcl::io::savePLYFile("ok.ply", *cloud);
//	   //delete (PointCloudEncoder);
//  
//	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
//	/*pcl::VoxelGrid<pcl::PointXYZRGB> sor;
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.01f, 0.01f, 0.01f);
//  sor.filter (*cloud_filtered);*/
//	cout << "start filtering" << endl;
//	/*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
//  sor.setInputCloud (cloud);
//  sor.setMeanK (50);
//  sor.setStddevMulThresh (1.0);
//  sor.filter (*cloud_filtered);*/
//
//
//	pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
//	PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();
//	//PointCloudDecoder->decodePointCloud(compressedData, cloud_filtered);
//
//	   cout << "done filtering" << endl;
//       
//    
//	  //workerThread.join();  
// 
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//	//viewer->setBackgroundColor (0, 0, 0);
//	
//	
//	 int v1(0);
//  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//  viewer->setBackgroundColor (0, 0, 0, v1);
//  viewer->addText("Radius: 1", 10, 10, "v1 text", v1);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);
//
//  int v2(0);
//  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
//  viewer->addText("Radius: 2", 10, 10, "v2 text", v2);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(tmp);
//  viewer->addPointCloud<pcl::PointXYZRGB> (tmp, rgb1, "sample cloud2", v2);
//
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
//  viewer->addCoordinateSystem (1.0);
//  viewer->resetCameraViewpoint("sample cloud2");
//       while (!viewer->wasStopped ())
//	{
//		viewer->spinOnce (100);
//	   }
//    std::cout << "main: done" << std::endl; 
//	
//	
//    return 0;
//}

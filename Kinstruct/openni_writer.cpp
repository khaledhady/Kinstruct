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
pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr globalcloud;
boost::mutex saveCloud;
int noFrames = 0;
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
int window = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr one (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr two (new pcl::PointCloud<pcl::PointXYZRGB>);
std::stringstream compressedData;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
bool update;
boost::mutex updateModelMutex;

void cloudToMat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat *color, cv::Mat *depth)
{
	for(int i = 0; i < color->cols; i++)
	{
		
		for(int j = 0; j < color->rows; j++)
		{
			pcl::PointXYZRGB point = cloud->at(i,j);
			   if (!isFinite (point))
			        continue;
			
			color->at<cv::Vec3b>(j, i)[0] = point.b;
			color->at<cv::Vec3b>(j, i)[1] = point.g;
			color->at<cv::Vec3b>(j, i)[2] = point.r;
			depth->at<float>(j, i) = point.z;
			
		}

	}
	cv::imshow("here", *color);
}

void
detectNew (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB)
{
  srand ((unsigned int) time (NULL));

  // Octree resolution - side length of octree voxels
  float resolution = 64.0f;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);


  // Add points from cloudA to octree
  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers ();

  // Add points from cloudB to octree
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();

  std::vector<int> newPointIdxVector;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Output points
  
  int index = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB> );
  tmp->resize(newPointIdxVector.size ());
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (size_t i = 0; i < newPointIdxVector.size (); ++i)
  {
	tmp->points[index].x = cloudB->points[newPointIdxVector[i]].x;
	tmp->points[index].y = cloudB->points[newPointIdxVector[i]].y;
	tmp->points[index].z = cloudB->points[newPointIdxVector[i]].z;
	tmp->points[index].r = cloudB->points[newPointIdxVector[i]].r;
	tmp->points[index].g = cloudB->points[newPointIdxVector[i]].g;
	tmp->points[index].b = cloudB->points[newPointIdxVector[i]].b;
	index++;
  }
  *cloudA += *tmp;

}
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
	

	cv::Mat img_matches;
	drawMatches( grayA, keypointsA, grayB, keypointsB,
               matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  
    //-- Show detected matches
    cv::imshow( "Good Matches" , img_matches );
	//window++;

	vector<cv::Point2f> selPoints1, selPoints2;
	cv::KeyPoint::convert(keypointsA, selPoints1);
	cv::KeyPoint::convert(keypointsB, selPoints2);

	vector<cv::Point3f> setA, setB;
	for(int i = 0; i < matches.size(); i++)
	{
		cv::Point2f point2DA = selPoints1.at(matches.at(i).queryIdx);
		cv::Point2f point2DB = selPoints2.at(matches.at(i).trainIdx);
		
		pcl::PointXYZRGB pointA = cloud->at(point2DA.x, point2DA.y);
		pcl::PointXYZRGB pointB = cloud2->at(point2DB.x, point2DB.y);
		  if (!isFinite (pointA) || !isFinite (pointB))
			    continue;
		  cout << pointA.z << endl; 

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
	
}

void transform(Transformation *transform, pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target)
{
	int pixelIndex = 0;
	target->points.resize (src->width * src->height);
	target->width = src->width;
	target->height = src->height;

	for (int k = 0; k < src->size() ; k++) {
		
			
			//int bit0 = depth->at<cv::Vec3b>(k,m)[0];
			//int bit1 = depth->at<cv::Vec3b>(k,m)[1];
		pcl::PointXYZRGB point = src->points[k];
			   if (!isFinite (point))
			        continue;
			float x = src->points[k].x;
			float y = src->points[k].y;
			float z = src->points[k].z;
				
			int blue = src->points[k].b;
			int green = src->points[k].g;
			int red = src->points[k].r;

			cv::Point3d myPoint(x, y, z);
			transform->applyToPoint(&myPoint);

			target->points[pixelIndex].x = myPoint.x;
			target->points[pixelIndex].y = myPoint.y;
			target->points[pixelIndex].z = myPoint.z;
			uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
              static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
			target->points[pixelIndex].rgb = *reinterpret_cast<float*>(&rgb);
			
			pixelIndex++;

			
	}

	

}


int main1()
{
	
	cout << "read input" << endl;
	cv::Mat imgA(480, 640, CV_8UC3 );
	cv::Mat imgB(480, 640, CV_8UC3 );
	cv::Mat depthA(480, 640, CV_32F );
	cv::Mat depthB(480, 640, CV_32F );
	pcl::PassThrough<pcl::PointXYZRGB> pass; // can do this without parameters
pass.setInputCloud( cloud );
pass.filter( *one );
pass.setInputCloud( cloud2 );
pass.filter( *two );
	cloudToMat(cloud, &imgA, &depthA);
	cv::waitKey(0);
	cloudToMat(cloud2, &imgB, &depthB);
	cv::waitKey(0);
//
	    Transformation *inverse = new Transformation(false);
		alignFrames(&imgA, &imgB, &depthA, &depthB, inverse);
		cv::waitKey(0);
		//global->concatenate(inverse);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZRGB>);
		transform(inverse, two, tmp);
		*cloud += *tmp;
		//inverse->applyToFrame(&imgB, &depthB, tmp);
		
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	
	 int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 1", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 2", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(tmp);
  viewer->addPointCloud<pcl::PointXYZRGB> (tmp, rgb1, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);
  viewer->resetCameraViewpoint("sample cloud1");
       while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	   }
    std::cout << "main: done" << std::endl; 
	
	
    return 0;
}



void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	  
	if (event.getKeySym () == "r" && event.keyDown ())
	{
		boost::mutex::scoped_lock saveLock(saveCloud);
		std::cout << "r was pressed => removing all text" << std::endl;
			pcl::PassThrough<pcl::PointXYZRGB> pass; // can do this without parameters
			PointCloud<pcl::PointXYZRGB>::Ptr deep_copy (new PointCloud<pcl::PointXYZRGB>( *globalcloud ) );
		if(noFrames == 0)
		{
			
			cloud = deep_copy;

		}
		else{
			cloud2 = deep_copy;
			main1();
		}
		noFrames++;
		/*char fileName[20];
		itoa(noFrames, fileName, 10);*/

		std::string fileName = ".pcd";
		std::stringstream tmp;
		tmp << noFrames << ".pcd";
		
		//pcl::io::savePCDFileASCII(fileName, *globalcloud);
		pcl::io::savePCDFileBinary(tmp.str() , *globalcloud);

		saveLock.unlock();

	char str[512];
	}
}

	

class SimpleOpenNIViewer
{
public:
	int noFrames;
	SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {  }
	

	

	void save()
	{
		/*char fileName[20];
		itoa(noFrames, fileName, 10);*/

		/*pcl::io::savePCDFileASCII (, cloud);
		std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;*/
		//pcl::io::savePCDFileASCII (fileName, *cloud);
	}

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{

		boost::mutex::scoped_lock saveLock(saveCloud);
		globalcloud = cloud;
		saveLock.unlock();
		
		if (!viewer.wasStopped())
		{
		 viewer.showCloud (cloud);
		}

	}

	void run ()
	{
		pcl::Grabber* x = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

		x->registerCallback (f);

		viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)&viewer);

		//viewer.registerKeyboardCallback(&SimpleOpenNIViewer::keyboardEventOccurred);
		x->start ();

		while (!viewer.wasStopped())
		{
			sleep (1);
		}

		x->stop ();
	}


	




	pcl::visualization::CloudViewer viewer;
};

int main ()
{
	SimpleOpenNIViewer v;
	v.run ();
	return 0;
}



//
//
//
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <boost/thread.hpp>
//#include <boost/date_time.hpp>
//#include <pcl/registration/icp.h>
//#include <pcl/filters/passthrough.h>
//
//#ifdef WIN32
//# define sleep(x) Sleep((x)*1000) 
//#endif
//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr globalCloud;
//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr first;
//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr second;
////pcl::PointCloud<pcl::PointXYZRGB>::Ptr firstCloud;
////pcl::PointCloud<pcl::PointXYZRGB>::Ptr secondCloud;
//boost::mutex saveCloud;
//int noFrames = 0;
//
//
//
//bool update2;
//boost::mutex updateModelMutex2;
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
//void visualize2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)  
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
//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
//{
//	  
//	if (event.getKeySym () == "r" && event.keyDown ())
//	{
//		std::cout << "r was pressed => removing all text" << std::endl;
//		
//		
//
//		boost::mutex::scoped_lock saveLock(saveCloud);
//		if(noFrames == 0)
//			first = globalCloud ; 
//		else if(noFrames == 1)
//		{
//			second = globalCloud ;
//
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr firstCloud (new pcl::PointCloud<pcl::PointXYZRGB>( *first ) );
//		    pcl::PointCloud<pcl::PointXYZRGB>::Ptr secondCloud (new pcl::PointCloud<pcl::PointXYZRGB>( *second ) );
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr firstCloudClean (new pcl::PointCloud<pcl::PointXYZRGB>( *first ) );
//		    pcl::PointCloud<pcl::PointXYZRGB>::Ptr secondCloudClean (new pcl::PointCloud<pcl::PointXYZRGB>( *second ) );
//			pcl::PassThrough<pcl::PointXYZRGB> pass; // can do this without parameters
//			pass.setInputCloud( firstCloud );
//			pass.filter( *firstCloudClean );
//			pass.setInputCloud( secondCloud );
//			pass.filter( *secondCloudClean );
//
//		
//			/*Eigen::Matrix4f tranfromationMAt2 ; 
//			  pcl::registration::TransformationEstimationSVD< pcl::PointXYZRGB ,  pcl::PointXYZRGB > SVNTransformer ; 
//			  SVNTransformer.estimateRigidTransformation( *firstCloud , *secondCloud , tranfromationMAt2);*/
//
//			Eigen::Matrix4f tranfromationMAt2  = getICPTransformation(firstCloudClean , secondCloudClean);
//			  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//			  pcl::transformPointCloud(*secondCloud , *transformed , tranfromationMAt2 );
//
//			  
//			  *firstCloud = *firstCloud +  *transformed ; 
//
//			  std::cout << tranfromationMAt2 << std::endl;
//
//			  visualize2(firstCloud);
//
//		}
//
//		saveLock.unlock();
//		noFrames++;
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
//		globalCloud = cloud;
//		saveLock.unlock();
//		
//		if (!viewer.wasStopped())
//		{
//		 viewer.showCloud (cloud);
//		}
//
//	}
//
//	void run ()
//	{
//		pcl::Grabber* interface = new pcl::OpenNIGrabber();
//
//		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
//			boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
//
//		interface->registerCallback (f);
//
//		viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)&viewer);
//
//		//viewer.registerKeyboardCallback(&SimpleOpenNIViewer::keyboardEventOccurred);
//		interface->start ();
//
//		while (!viewer.wasStopped())
//		{
//			sleep (1);
//		}
//
//		interface->stop ();
//	}
//
//
//
//	pcl::visualization::CloudViewer viewer;
//};
//
//int main ()
//{
//	SimpleOpenNIViewer v;
//	v.run ();
//	return 0;
//}
//
//
//
//
//

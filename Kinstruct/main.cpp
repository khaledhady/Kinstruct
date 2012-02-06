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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultColored (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastAligned (new pcl::PointCloud<pcl::PointXYZRGB>);
Transformation *global = new Transformation(true);
Eigen::Matrix4f lastTransformation;
std::stringstream compressedData;
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


void alignFrames(cv::Mat *colorA, cv::Mat *colorB, cv::Mat *depthA, cv::Mat *depthB, Transformation *inverse,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB)
{
	// Prepare the matcher
	cv::Mat grayA, grayB;
	cv::cvtColor(*colorA, grayA, CV_RGB2GRAY);
	cv::cvtColor(*colorB, grayB, CV_RGB2GRAY);

	/*FeatureTracker tracker;
	tracker.track(*colorA, *colorB);
	std::vector<cv::Point2f> pointsA = tracker.initial;
	std::vector<cv::Point2f> pointsB = tracker.points[1];*/

	RobustMatcher rmatcher;

	cv::Ptr<cv::FeatureDetector> pfd = new cv::SurfFeatureDetector(100);

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
    cv::imshow( "Good Matches1" , img_matches);
	//cv::imshow( "Good Matches2" , *colorB );
	//cv::waitKey(0);
	//window++;

	vector<cv::Point2f> selPoints1, selPoints2;
	cv::KeyPoint::convert(keypointsA, selPoints1);
	cv::KeyPoint::convert(keypointsB, selPoints2);

	vector<cv::Point3f> setA, setB;

	/*for(int i = 0; i < pointsA.size() ; i++)
	{
		cv::Point2f point2DA = pointsA.at(i);
		cv::Point2f point2DB = pointsB.at(i);

		pcl::PointXYZRGB pointA = cloudA->at(point2DA.x, point2DA.y);
		pcl::PointXYZRGB pointB = cloudB->at(point2DB.x, point2DB.y);
		  if (!isFinite (pointA) || !isFinite (pointB))
			    continue;
		  cout << pointA.z << endl; 

		cv::Point3f point3D (pointA.x, pointA.y, pointA.z); 
		setA.push_back(point3D);
		cv::Point3f point3DB (pointB.x, pointB.y, pointB.z); 
		setB.push_back(point3DB);
	}*/

	for(int i = 0; i < matches.size(); i++)
	{
		cv::Point2f point2DA = selPoints1.at(matches.at(i).queryIdx);
		cv::Point2f point2DB = selPoints2.at(matches.at(i).trainIdx);

		pcl::PointXYZRGB pointA = cloudA->at(point2DA.x, point2DA.y);
		pcl::PointXYZRGB pointB = cloudB->at(point2DB.x, point2DB.y);
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

   
void visualize()  
{  


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor (0, 0, 0, v1);
	viewer->addText("Result in RGB", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(result);
	viewer->addPointCloud<pcl::PointXYZRGB> (result, rgb, "result", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
	viewer->addText("Result Colored", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbColored(resultColored);
	viewer->addPointCloud<pcl::PointXYZRGB> (resultColored, rgbColored, "resultColored", v2);

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "resultColored");
	viewer->addCoordinateSystem (1.0);


	/*boost::mutex::scoped_lock drawLock(updateModelMutex);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(result);
	viewer->addPointCloud<pcl::PointXYZRGB> (result, rgb, "sample cloud");
	drawLock.unlock();
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);*/
	//viewer->resetCameraViewpoint("sample cloud");
	int i = 0;
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);


		boost::mutex::scoped_lock updateLock(updateModelMutex);
		/*if(updateLock.try_lock())*/
		
		if(update)
		{
			cout << update << endl;
			stringstream tmp;
			tmp << i;
			viewer->removePointCloud("result", v1);
			
			//viewer->addPointCloud<pcl::PointXYZRGB>(result, rgb, tmp.str());
			//viewer->updatePointCloud(
			viewer->addPointCloud<pcl::PointXYZRGB> (result, rgb, "result", v1);
			viewer->removePointCloud("resultColored", v2);
			viewer->addPointCloud<pcl::PointXYZRGB> (resultColored, rgbColored, "resultColored", v2);
			//viewer->updatePointCloud<pcl::PointXYZRGB>(result, rgb, "sample cloud");
			if(i == 0)
			{
				viewer->resetCameraViewpoint("result");
				//viewer->resetCameraViewpoint("resultColored");
			}
			update = false;
			i++;
		}
		updateLock.unlock();
     
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}   
    std::cout << "Worker: finished" << std::endl;  
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
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*downsampled);
  std::cerr << "PointCloud after filtering: " << downsampled->width * downsampled->height 
       << " data points (" << pcl::getFieldsList (*downsampled) << ").";
}

int main()
{

	int frameNo = START_FRAME;
	int times = 1;
	int final = 3;
	boost::thread workerThread(visualize);
	while(times < final)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredA (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredB (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledA (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledB (new pcl::PointCloud<pcl::PointXYZRGB>);

		std::stringstream streamA;
		streamA << "desk" << times << ".pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (streamA.str(), *cloudA) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}
		times++;
		std::stringstream streamB;
		streamB << "desk" << times << ".pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (streamB.str(), *cloudB) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}

		if(times == 2)
		{
			boost::mutex::scoped_lock initializeLock(updateModelMutex);
			update = true;
			*result += *cloudA;
			downsample(cloudA, lastAligned);
			initializeLock.unlock();
		}

		cv::Mat imgA(cloudA->height, cloudA->width, CV_8UC3 );
		cv::Mat imgB(cloudB->height, cloudB->width, CV_8UC3 );
		cv::Mat depthA(cloudA->height, cloudA->width, CV_32F );
		cv::Mat depthB(cloudB->height, cloudB->width, CV_32F );

		pcl::PassThrough<pcl::PointXYZRGB> pass; 
		pass.setInputCloud( cloudA );
		pass.filter( *coloredA );
		pass.setInputCloud( cloudB );
		pass.filter( *coloredB );

		cloudToMat(cloudA, &imgA, &depthA);
		//cv::waitKey(0);
		cloudToMat(cloudB, &imgB, &depthB);
		//cv::waitKey(0);
	    Transformation inverse(false);
		alignFrames(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB);
		//cv::waitKey(0);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedDownsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
		//if(times == 2)
			//global->concatenate(&inverse);
		cout << "getting ICP \n";
		time_t before;
		before = time (NULL);
		Eigen::Matrix4f initialTransformation;
		if(times != 2)
			inverse.concatenate(&lastTransformation);
		inverse.get4X4Matrix(&initialTransformation);
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		//boost::mutex::scoped_lock downsamplingLock(updateModelMutex);
		//downsample(result, downsampledA);
		//downsamplingLock.unlock();
		downsample(coloredB, downsampledB);
		icp.setInputCloud(downsampledB);
		icp.setInputTarget(lastAligned);
  
		icp.setMaxCorrespondenceDistance(0.5);
		//icp.setRANSACOutlierRejectionThreshold(0.05);
		//icp.setTransformationEpsilon(0.000001);
		icp.setEuclideanFitnessEpsilon(0.001);
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
		pcl::transformPointCloud(*coloredB, *transformed, icp.getFinalTransformation());

		boost::mutex::scoped_lock updateLock(updateModelMutex);
		update = true;
		//detectNew(result, transformed);
		*result += *transformed;
		updateLock.unlock();
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
		
		*resultColored += *transformed;
	}
    
	 /*pcl::io::savePCDFileASCII ("test.pcd", *cloud);
  cout << "Saved " << cloud->points.size () << " data points to test_pcd.pcd." << std::endl;*/
	//Visualizer::start(argc, argv);
	 
	workerThread.join();  
    
 
	/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	 int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Result in RGB", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(result);
  viewer->addPointCloud<pcl::PointXYZRGB> (result, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Result Colored", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(resultColored);
  viewer->addPointCloud<pcl::PointXYZRGB> (resultColored, rgb1, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);
  viewer->resetCameraViewpoint("sample cloud1");
       while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	   }
    std::cout << "main: done" << std::endl; 
	*/

    return 0;
}
#include "Commons.h"
#include "RobustMatcher.h"
#include "HornMethod.h"
#include <math.h>
#include <time.h>
#include <boost/thread.hpp>  
#include <boost/date_time.hpp>  
#include "pcl/win32_macros.h"

using namespace std;
using namespace pcl;
int window = 0;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultColored (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr lastAligned (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PolygonMesh triangles;
bool mesh = false;
//Transformation *global = new Transformation(true);
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
			pcl::PointXYZRGB point = cloud->at(i + 50, j + 50);
			   if (!isFinite (point))
			        continue;

			color->at<cv::Vec3b>(j, i)[0] = point.b;
			color->at<cv::Vec3b>(j, i)[1] = point.g;
			color->at<cv::Vec3b>(j, i)[2] = point.r;
			depth->at<float>(j, i) = point.z;


		}
	}
}


void refineSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

	time_t beforeSmooth;
	beforeSmooth = time (NULL);

// Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // Output has the same type as the input one, it will be only smoothed
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points (new pcl::PointCloud<pcl::PointXYZRGB> ());

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::Normal> mls;
  //============================================================================
  //OutlierRemoval 
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud);

  //************************************************
  //Outliers removal but commented due to this error
  //************************************************

  //sor.setMeanK (50);
  //sor.setStddevMulThresh (0.01);
  //sor.setNegative (true);
  //sor.filter (*cloud);
			
  //=============================================================================

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setSearchMethod(tree);
  mls.setSearchRadius (0.01);
  mls.setPolynomialFit (true);

  // Reconstruct
  mls.reconstruct (*mls_points);  // takes 1 minute for 2 pointclouds

  time_t afterSmooth;
  afterSmooth = time (NULL);
  cout << "Smoothing Time " << afterSmooth - beforeSmooth << endl;
  *cloud = *mls_points ; 
  mls_points->clear();

  
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

	if(tracked > 100 || tracked < 10)
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

void downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled)
{
  // Create the filtering object
  std::cerr << "PointCloud before filtering: " << original->width * original->height 
       << " data points (" << pcl::getFieldsList (*original) << ").";
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (original);
  sor.setLeafSize (0.008f, 0.008f, 0.008f);
  sor.filter (*downsampled);
  std::cerr << "PointCloud after filtering: " << downsampled->width * downsampled->height 
       << " data points (" << pcl::getFieldsList (*downsampled) << ").";
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
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbColored(resultColored);
	//viewer->addPointCloud<pcl::PointXYZRGB> (resultColored, rgbColored, "resultColored", v2);
	

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "result");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "resultColored");
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
		if(mesh)
		{
			viewer->addPolygonMesh(triangles, "mesh", v2);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 1, "mesh", v2);
			//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, "mesh", v2);
			mesh = false;
		}

		if(update)
		{
			cout << update << endl;
			stringstream tmp;
			tmp << i;
			//viewer->removePointCloud("result", v1);
			
			//viewer->addPointCloud<pcl::PointXYZRGB>(result, rgb, tmp.str());
			//viewer->updatePointCloud(
			
			viewer->addPointCloud<pcl::PointXYZRGB> (result, rgb, tmp.str(), v1);
			//viewer->removePointCloud("resultColored", v2);
			tmp << "color";
			//viewer->addPointCloud<pcl::PointXYZRGB> (resultColored, rgbColored, tmp.str(), v2);
			//viewer->addPolygonMesh(triangles, tmp.str(), v2);
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

int main()
{

	int times = 1;
	int final = 10;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredA (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedDownsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledA (new pcl::PointCloud<pcl::PointXYZRGB>);
	std::stringstream streamA;
	streamA << "" << times << ".pcd";
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (streamA.str(), *cloudA) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	boost::mutex::scoped_lock initializeLock(updateModelMutex);
	update = true;
	*result += *cloudA;
	downsample(cloudA, lastAligned);
	initializeLock.unlock();
	cv::Mat imgA(cloudA->height - 50, cloudA->width - 50, CV_8UC3 );
	cv::Mat depthA(cloudA->height - 50, cloudA->width - 50, CV_32F );
	pcl::PassThrough<pcl::PointXYZRGB> pass; 
	pass.setInputCloud( cloudA );
	pass.filter( *coloredA );
	cloudToMat(cloudA, &imgA, &depthA);
	boost::thread workerThread(visualize);
    double total = 0;
	bool first = true;
	while(times < final)
	{
		times++;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredB (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledB (new pcl::PointCloud<pcl::PointXYZRGB>);
		
		std::stringstream streamB;
		streamB << "" << times << ".pcd";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (streamB.str(), *cloudB) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}
		time_t before;
		before = time (NULL);

		cv::Mat imgB(cloudB->height - 50, cloudB->width - 50, CV_8UC3 );
		cv::Mat depthB(cloudB->height - 50, cloudB->width - 50, CV_32F );

		pass.setInputCloud( cloudB );
		pass.filter( *coloredB );

		//cv::waitKey(0);
		cloudToMat(cloudB, &imgB, &depthB);
		//cv::waitKey(0);
	    Transformation inverse(false);
		// use the two obtained frames to get the initial tansformations
			bool ignore  = alignFrames(&imgA, &imgB, &depthA, &depthB, &inverse, cloudA, cloudB);

			// ignore is true if the two point clouds are same, so just continue and discard cloudB
			// ignore can also be true if the two points clouds are far so tracking has failed, in that
			// case discard the frame and wait till the user gets back to a frame that can be tracked
			if(!ignore )
			{
				
				cout << "getting ICP \n";
				time_t before;
				before = time (NULL);
				Eigen::Matrix4f initialTransformation;
				if(!first)
					inverse.concatenate(&lastTransformation);
				first = false;
				inverse.get4X4Matrix(&initialTransformation);

				// Perform ICP on the downsampled point clouds
				pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
				downsample(coloredB, downsampledB);
				icp.setInputCloud(downsampledB);
				icp.setInputTarget(lastAligned);
				icp.setMaxCorrespondenceDistance(0.1);
				//icp.setRANSACOutlierRejectionThreshold(0.05);
				//icp.setTransformationEpsilon(1e-6);
				icp.setEuclideanFitnessEpsilon(0.0001);
				icp.setMaximumIterations(100);
				transformedDownsampled->clear();
				icp.align(*transformedDownsampled, initialTransformation);
				lastAligned->clear();
				pcl::copyPointCloud(*transformedDownsampled, *lastAligned);
				std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
				time_t after = time (NULL);
				cout << "Found Transformation " << after - before << endl;
				lastTransformation = icp.getFinalTransformation();
				boost::mutex::scoped_lock updateLock(updateModelMutex);
				result->clear();
				pcl::transformPointCloud(*coloredB, *result, icp.getFinalTransformation());
				/*resultColored->clear();
				pcl::copyPointCloud(*result, *resultColored);*/
				
				update = true;
				*global += *result;
				updateLock.unlock();
				cloudA = cloudB;
				coloredA = coloredB;
				imgB.copyTo(imgA);
				depthB.copyTo(depthA);
			}
			else cout << "Frame is too close ... ignoring" << endl;
	}

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
			  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			  tree->setInputCloud (global);
			  n.setInputCloud (global);
			  n.setSearchMethod (tree);
			  n.setKSearch (20);
			  n.compute (*normals);
			  //* normals should not contain the point normals + surface curvatures

			  // Concatenate the XYZ and normal fields*
			  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			  pcl::concatenateFields (*global, *normals, *cloud_with_normals);
			  //* cloud_with_normals = cloud + normals

			  // Create search tree*
			  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
			  tree2->setInputCloud (cloud_with_normals);

			  // Initialize objects
			  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
			  

			  // Set the maximum distance between connected points (maximum edge length)
			  gp3.setSearchRadius (0.025);

			  // Set typical values for the parameters
			  gp3.setMu (2.5);
			  gp3.setMaximumNearestNeighbors (100);
			  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			  gp3.setMinimumAngle(M_PI/18); // 10 degrees
			  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			  gp3.setNormalConsistency(true);

			  // Get result
			  gp3.setInputCloud (cloud_with_normals);
			  gp3.setSearchMethod (tree2);
			  gp3.reconstruct (triangles);
			  mesh = true;
			 // pcl::io::savePLYFile ("mesh.vtk", triangles);
	 cout << "Total of all frames " << total << endl;
	workerThread.join();  
    
 

    return 0;
}
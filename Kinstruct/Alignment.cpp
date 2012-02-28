#include "Alignment.h"

Alignment::Alignment()
{
}

// Function takes two color and depth frames and tracks interesting points from the first frame
// to the second frame, then use Horn's method to find a rigid transformation between the 
// first and the second, this transformation is then inversed.
bool Alignment::getInitialTransformation(cv::Mat *colorA, cv::Mat *colorB, cv::Mat *depthA, cv::Mat *depthB, Transformation *inverse,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB)
{
	Tracker tracker;
	//cout << "Tracking.. " << endl ;
	time_t before;
    before = time (NULL);

	// Copy images as they will be marked
	cv::Mat imageA, imageB;
	colorA->copyTo(imageA);
	colorB->copyTo(imageB);

	// Track
    int tracked = tracker.track(imageA, imageB);

	time_t after;

	after = time (NULL);
	//cout << "Finished Tracking " << after - before << endl;

	cv::imshow( "Current keyframe " , imageA);
	cvMoveWindow( "Current keyframe ", 0, 400);
	cv::imshow( "Online " , imageB );
	cvMoveWindow( "Online ", 600, 400 );
	cv::waitKey(30);

	// If number of tracked > 70 then the new frame is almost the old one, so no
	// need to put it as it will add very little info, also if tracked is less than
	// 10 then we can't produce good results
	if(tracked > 70 || tracked < 10)
		return false;

	// Collect the successfully tracked points with their z indices to be used
	// by horn method
	vector<cv::Point3f> setA, setB;
	for(int i = 0; i < tracker.initial.size() ; i++)
	{
		cv::Point2f point2DA = tracker.initial.at(i);
		cv::Point2f point2DB = tracker.final.at(i);
		if(point2DA.x < 0 || point2DA.y < 0 || point2DB.x < 0 || point2DB.y < 0)
			continue;
		pcl::PointXYZRGB pointA = cloudA->at(point2DA.x, point2DA.y);
		pcl::PointXYZRGB pointB = cloudB->at(point2DB.x, point2DB.y);

		if (!isFinite (pointA) || !isFinite (pointB))
			continue;

		cv::Point3f point3D (pointA.x, pointA.y, pointA.z); 
		setA.push_back(point3D);
		cv::Point3f point3DB (pointB.x, pointB.y, pointB.z); 
		setB.push_back(point3DB);
	}

	// Use horn method to get the transformation
	//cout << "getting transformation \n";
	before = time (NULL);
	Transformation result(false);
	HornMethod hornMethod;
	hornMethod.getTransformation(&setA, &setB, &result);
	inverse->invert(&result);
	after = time (NULL);
	//cout << "Found transformation " << after - before << endl;

	// meaning move this frame to the next step (ICP)
	return true;
}

void Alignment::downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr original, pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled)
{
  // Create the filtering object
  //std::cerr << "PointCloud before filtering: " << original->width * original->height 
    //   << " data points (" << pcl::getFieldsList (*original) << ").";
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (original);
  sor.setLeafSize (0.05f, 0.05f, 0.05f);
  sor.filter (*downsampled);
  //std::cerr << "PointCloud after filtering: " << downsampled->width * downsampled->height 
     //  << " data points (" << pcl::getFieldsList (*downsampled) << ").";
}


// Converts an organized point cloud to a color image and a depth image
void Alignment::cloudToMat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat *color, cv::Mat *depth)
{
	for(int i = 0; i < color->cols; i++)
	{
		for(int j = 0; j < color->rows; j++)
		{
			pcl::PointXYZRGB point = cloud->at(i + 30, j + 30);

			// Ignore any unidentified point
			if (!isFinite (point))
			{
			    color->at<cv::Vec3b>(j, i)[0] = 0;
				color->at<cv::Vec3b>(j, i)[1] = 0;
				color->at<cv::Vec3b>(j, i)[2] = 0;
				depth->at<float>(j, i) = 0;
			}else
			{
				color->at<cv::Vec3b>(j, i)[0] = point.b;
				color->at<cv::Vec3b>(j, i)[1] = point.g;
				color->at<cv::Vec3b>(j, i)[2] = point.r;
				depth->at<float>(j, i) = point.z;
			}
		}
	}
}
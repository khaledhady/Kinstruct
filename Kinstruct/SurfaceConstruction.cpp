#include "SurfaceConstruction.h"
SurfaceConstruction::SurfaceConstruction()
{
	pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	// instantiate point cloud compression for encoding and decoding
	pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (compressionProfile, true);
	pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();
}

void SurfaceConstruction::compressPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr uncompressed, pcl::PointCloud<pcl::PointXYZRGB>::Ptr compressed)
{
	compressed->clear();
	std::stringstream compressedData;

	// compress point cloud
	PointCloudEncoder->encodePointCloud (uncompressed, compressedData);

	// decompress point cloud
	PointCloudDecoder->decodePointCloud (compressedData, compressed);
}


void SurfaceConstruction::refineSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
  sor.filter (*mls_points);
  cloud->clear();
  pcl::copyPointCloud(*mls_points, *cloud);

  //************************************************
  //Outliers removal but commented due to this error
  //************************************************

  //sor.setMeanK (50);
  //sor.setStddevMulThresh (0.01);
  //sor.setNegative (true);
  //sor.filter (*cloud);
			
  //=============================================================================

  // Set parameters
  //mls.setInputCloud (cloud);
  //mls.setSearchMethod(tree);
  //mls.setSearchRadius (0.01);
  //mls.setPolynomialFit (true);

  //// Reconstruct
  //mls.reconstruct (*mls_points);  // takes 1 minute for 2 pointclouds

  //time_t afterSmooth;
  //afterSmooth = time (NULL);
  //cout << "Smoothing Time " << afterSmooth - beforeSmooth << endl;
  //*cloud = *mls_points ; 
  //mls_points->clear();

  
}

void SurfaceConstruction::fastTranguilation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PolygonMesh triangles;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
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
	pcl::io::saveVTKFile ("mesh.vtk", triangles);
	//mesh = true;
	//pcl::io::savePLYFile ("mymesh.ply", triangles);
}

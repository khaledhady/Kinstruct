#include "SurfaceConstruction.h"
SurfaceConstruction::SurfaceConstruction()
{
	
}

void SurfaceConstruction::compressPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr uncompressed, pcl::PointCloud<pcl::PointXYZRGB>::Ptr compressed)
{
	compressed->clear();
	std::stringstream compressedData;
	pcl::octree::compression_Profiles_e compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	// instantiate point cloud compression for encoding and decoding
	pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> (compressionProfile, true);
	pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder = new pcl::octree::PointCloudCompression<pcl::PointXYZRGB> ();

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

template<typename T>
class PCLMarchingCubesGreedyWrapper : public pcl::MarchingCubesGreedy<T>
{ 
        public: 
                PCLMarchingCubesGreedyWrapper() {}; 
                virtual ~PCLMarchingCubesGreedyWrapper() {}; 

        protected: 
                virtual void performReconstruction(pcl::PointCloud<T    >&, std::vector<pcl::Vertices>&) 
                { 
                        std::cout << "IS THIS EVER USED?" << std::endl; 
                } 
}; 

//typedef pcl::PointXYZ PointType; 
typedef pcl::PointXYZRGB PointType; 
typedef pcl::Normal Normals;	
//typedef pcl::PointNormal PointTypeNormal; 
typedef pcl::PointXYZRGBNormal PointTypeNormal; 

void SurfaceConstruction::fastTranguilation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	double leafSize = 0.05; 
        float isoLevel = 0.5; 

        // Load input file */ 
        
        pcl::PointCloud<PointTypeNormal>::Ptr pointcloudNormal (new pcl::PointCloud<PointTypeNormal> ()); 

        /* Marching Cubes Reconstruction */ 
        pcl::PolygonMesh mesh; 

        // Normal estimation* 
        pcl::NormalEstimation<PointType, PointTypeNormal> norm_est; 
        pcl::PointCloud<Normals>::Ptr normals (new pcl::PointCloud<Normals>); 
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>); 
        tree->setInputCloud (cloud); 
        norm_est.setInputCloud (cloud); 
        norm_est.setSearchMethod (tree); 
        norm_est.setKSearch(30); 
        norm_est.compute(*pointcloudNormal); 
        pcl::copyPointCloud (*cloud, *pointcloudNormal); 

        // Create the search method 
        pcl::search::KdTree<PointTypeNormal>::Ptr tree2 (new pcl::search::KdTree<PointTypeNormal>); 
        tree2->setInputCloud (pointcloudNormal); 
        // Initialize objects 
        PCLMarchingCubesGreedyWrapper<PointTypeNormal> mc; 
        // Set parameters 
        mc.setLeafSize(leafSize);   
        mc.setIsoLevel(isoLevel);   //ISO: must be between 0 and 1.0 
        mc.setSearchMethod(tree2); 
        mc.setInputCloud(pointcloudNormal); 
        // Reconstruct 
        mc.reconstruct (mesh); 

        //Saving to disk in VTK format: 
        pcl::io::saveVTKFile ("nice.vtk", mesh); 
        pcl::io::savePLYFile ("nice.ply", mesh); 
}

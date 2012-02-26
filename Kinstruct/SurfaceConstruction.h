#include "Commons.h"

#ifndef SURFACE
#define SURFACE


class SurfaceConstruction
{
	public:
		SurfaceConstruction();
		void compressPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr uncompressed, pcl::PointCloud<pcl::PointXYZRGB>::Ptr compressed);
		void refineSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void fastTranguilation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	private:
	
		pcl::octree::compression_Profiles_e compressionProfile;
		// instantiate point cloud compression for encoding and decoding
		pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
		pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
};

#endif
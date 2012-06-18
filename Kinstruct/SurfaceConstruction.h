#include "Commons.h"

#ifndef SURFACE
#define SURFACE


class SurfaceConstruction
{
	public:
		SurfaceConstruction();
		void compressPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr uncompressed, pcl::PointCloud<pcl::PointXYZRGB>::Ptr compressed);
		void refineSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void marchingCubes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	private:
	
		pcl::octree::compression_Profiles_e compressionProfile;
		// instantiate point cloud compression for encoding and decoding
		pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder;
		pcl::octree::PointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder;
};

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

typedef pcl::PointXYZRGB PointType; 
typedef pcl::Normal Normals;	
typedef pcl::PointXYZRGBNormal PointTypeNormal; 

#endif
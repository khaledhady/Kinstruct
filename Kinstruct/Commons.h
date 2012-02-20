#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#ifndef COMMONS
#define COMMONS
	#include <pcl/point_types.h>
	#include <pcl/visualization/cloud_viewer.h>
	#include "pcl/octree/octree.h"
	#include <pcl/io/io.h>
	#include <pcl/io/pcd_io.h>
	#include <pcl/io/ply_io.h>
	#include <pcl/compression/octree_pointcloud_compression.h>
	#include <pcl/point_cloud.h>
	#include <pcl/kdtree/kdtree_flann.h>
	#include <pcl/surface/mls.h>
	#include <math.h>
	#include <vector>
	#include <time.h>
#endif

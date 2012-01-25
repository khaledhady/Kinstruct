#include <math.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class Transformation
{
	public:
		Transformation(bool identity);
		void setRotation(double s, double qx, double qy, double qz, bool normalizeQuaternion = false);
		void setTranslation(double x, double y, double z);
		void applyToPoint(cv::Point3d *point);
		void applyToPoint(cv::Point3f *p, cv::Point3f *pTransformed);
		void applyToFrame(cv::Mat *color, cv::Mat *depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void invert(const Transformation *t) ;
		void concatenate(const Transformation *concatenated);

		double rotation[3][3];
		double translation[3];
		

	private:
	/*	float rotation[3][3];
		float translation[3];*/
};
#include <math.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
using namespace cv;

class Transformation
{
	public:
		Transformation();
		void setRotation(double s, double qx, double qy, double qz, bool normalizeQuaternion = false);
		void setTranslation(double x, double y, double z);
		void applyToPoint(Point3d *point);
		void apply(Point3f *p, Point3f *pTransformed);
		void invert(const Transformation *t) ;
		void concatenate(const Transformation *concatenated);

		double rotation[3][3];
		double translation[3];
		

	private:
	/*	float rotation[3][3];
		float translation[3];*/
};
#include "Transformation.h"

Transformation::Transformation(bool identity)
{
	if(identity)
	{
		
		this->setRotation(1, 0, 0, 0, true);
				
		for(int i = 0; i < 3; i++)
			this->translation[i] = 0;
	}else
	{
		for(int i = 0; i < 3; i++)
			for(int j = 0; j < 3; j++)
				this->rotation[i][j] = 0;
		for(int i = 0; i < 3; i++)
			this->translation[i] = 0;
	}
}

void Transformation::setRotation(double s, double qx, double qy, double qz, bool normalizeQuaternion)
{
	if(normalizeQuaternion) {
		double norm = sqrt(s*s + qx*qx + qy*qy + qz*qz);
		s/=norm;
		qx/=norm;
		qy/=norm;
		qz/=norm;
	}

	this->rotation[0][0] = 1-2*(qy*qy+qz*qz);   
	this->rotation[0][1] = 2*(qx*qy-s*qz);    
	this->rotation[0][2] = 2*(qx*qz+s*qy);    
	
	this->rotation[1][0] = 2*(qx*qy+s*qz);     
	this->rotation[1][1] = 1-2*(qx*qx+qz*qz);  
	this->rotation[1][2] = 2*(qy*qz-s*qx);    
	
	this->rotation[2][0] = 2*(qx*qz-s*qy);     
	this->rotation[2][1] = 2*(qy*qz+s*qx);    
	this->rotation[2][2] = 1-2*(qx*qx+qy*qy);  
}

void Transformation::setTranslation(double x, double y, double z)
{
	this->translation[0] = x;
	this->translation[1] = y;
	this->translation[2] = z;
}

void Transformation::applyToPoint(cv::Point3d *point)
{
	point->x = this->rotation[0][0] * point->x +
			   this->rotation[0][1] * point->y +
			   this->rotation[0][2] * point->z +
			   this->translation[0];

	point->y = this->rotation[1][0] * point->x +
			   this->rotation[1][1] * point->y +
			   this->rotation[1][2] * point->z +
			   this->translation[1];

	point->z = this->rotation[2][0] * point->x +
			   this->rotation[2][1] * point->y +
			   this->rotation[2][2] * point->z +
			   this->translation[2];

}

void Transformation::applyToFrame(cv::Mat *color, cv::Mat *depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	int pixelIndex = 0;
	cloud->points.resize (color->rows * color->cols);

	for (int k = 0; k < color->rows ; k++) {
		for (int m = 0; m < color->cols; m++) {
			
			int bit0 = depth->at<cv::Vec3b>(k,m)[0];
			//int bit1 = depth->at<cv::Vec3b>(k,m)[1];
			//double z = (bit0 | bit1 << 8 );
				
			int blue = color->at<cv::Vec3b>(k,m)[0];
			int green = color->at<cv::Vec3b>(k,m)[1];
			int red = color->at<cv::Vec3b>(k,m)[2];

			cv::Point3d myPoint(m, k, bit0 * 3500 / 2550);
			this->applyToPoint(&myPoint);

			cloud->points[pixelIndex].x = myPoint.x;
			cloud->points[pixelIndex].y = myPoint.y;
			cloud->points[pixelIndex].z = myPoint.z;
			uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
              static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
			cloud->points[pixelIndex].rgb = *reinterpret_cast<float*>(&rgb);
			
			pixelIndex++;

			
		}
	}

	

}

void Transformation::applyToPoint(cv::Point3f *p, cv::Point3f *pTransformed)
{
	double x, y, z;

	x = this->rotation[0][0] * p->x + 
	    this->rotation[0][1] * p->y + 
	    this->rotation[0][2] * p->z + 
	    this->translation[0];
	y = this->rotation[1][0] * p->x + 
	    this->rotation[1][1] * p->y + 
	    this->rotation[1][2] * p->z + 
	    this->translation[1];
	z = this->rotation[2][0] * p->x + 
	    this->rotation[2][1] * p->y + 
	    this->rotation[2][2] * p->z + 
	    this->translation[2];
	pTransformed->x = x;
	pTransformed->y = y;
	pTransformed->z = z;
}

void Transformation::invert(const Transformation *t) 
{
	for(int i=0; i<3; i++)
	  for(int j=0; j<3; j++)
			this->rotation[i][j] = t->rotation[j][i];
	this->translation[0] = -(t->rotation[0][0] * t->translation[0] + 
			                     t->rotation[1][0] * t->translation[1] + 
			                     t->rotation[2][0] * t->translation[2]);
	this->translation[1] = -(t->rotation[0][1] * t->translation[0] + 
			                     t->rotation[1][1] * t->translation[1] + 
			                     t->rotation[2][1] * t->translation[2]);
	this->translation[2] = -(t->rotation[0][2] * t->translation[0] + 
			                     t->rotation[1][2] * t->translation[1] + 
			                     t->rotation[2][2] * t->translation[2]);
}

void Transformation::concatenate(const Transformation *concatenated) 
{
  int i,j;
  double tmpRotation[3][3];
  double tmpTranslation[3];
  
  for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			tmpRotation[i][j]=this->rotation[i][0]*concatenated->rotation[0][j] +
	                      this->rotation[i][1]*concatenated->rotation[1][j] +
	                      this->rotation[i][2]*concatenated->rotation[2][j];
		}
  }
  for(i=0; i<3; i++) 
		tmpTranslation[i] = this->rotation[i][0]*concatenated->translation[0] +
	                      this->rotation[i][1]*concatenated->translation[1] +
	                      this->rotation[i][2]*concatenated->translation[2] +
	                      this->translation[i];

	int sz = 3*sizeof(double);
	memcpy(this->translation,tmpTranslation,sz);
	memcpy(this->rotation[0],&(tmpRotation[0]),sz);
	memcpy(this->rotation[1],&(tmpRotation[1]),sz);
	memcpy(this->rotation[2],&(tmpRotation[2]),sz);
}


#ifndef _POINT3D_H_
#define _POINT3D_H_



#include <vnl/vnl_vector_ref.h> 

/**
 * 3D points.
 *
 * Author: Ziv Yaniv
 */


class Point3D {
public:
	/**
	 * Ouput the point data to the given output stream as the following string:
	 *[x1,x2,...xn]
	 */
	friend std::ostream &operator<<(std::ostream& output, const Point3D &p) {
		output<<"["<<p.data[0]<<", "<<p.data[1]<<", "<<p.data[2]<<"]";
		return output;
	}
  /**
	 * Default constructor, sets the point to zero.
	 */
	Point3D(){memset(this->data,0,3*sizeof(double));}

  /**
	 * Construct a point from the given pointer to an array.
	 * @param fillData An array that has at least three elements.
	 */
	Point3D(double *fillData) {memcpy(this->data,fillData,3*sizeof(double));}

  /**
	 * Copy constructor.
	 * @param other The point we copy.
	 */
	Point3D(const Point3D &other) {memcpy(this->data,other.data,3*sizeof(double));}

	/**
	 * Access to the coordinates of this point. No bounds checking is performed.
	 */
	double & operator[](int index) {return this->data[index];}
	
	/**
	 * Return this point as a vnl_vector_ref object.
	 */
	vnl_vector_ref<double> getVnlVector() {return vnl_vector_ref<double>(3, this->data);}

private:

	double data[3];
};

#endif //_POINT3D_H_
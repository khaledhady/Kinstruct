#ifndef _ABSOLUTE_ORIENTATION_H_
#define _ABSOLUTE_ORIENTATION_H_



#include "Frame.h"
#include "Point3D.h"


/*
 * Given the coordinates of 'n>=3' points in two coordinate systems
 * find the transformation between them.
 * We call the coordinate systems "left" and "right".
 * We seek the rigid transformation T such that T*P_left = P_right
 * This implementation is based on the paper "Closed-form solution of 
 * absolute orientation using unit quaternions", B.K.P. Horn,
 * Journal of the Optical Society of America, Vol. 4(4), pp 629--642, 1987.
 * 
 * @author: Ziv Yaniv 
 */
class AbsoluteOrientation {
public:
	/**
	 * Compute the rigid transformation so that sum(||right[i] - T*left[i]||^2) is minimal.
	 */
  static void compute(std::vector<Point3D> &left, std::vector<Point3D> &right, Frame &result);

  static void selfTest(std::ostream &out);
};


#endif //_ABSOLUTE_ORIENTATION_H_
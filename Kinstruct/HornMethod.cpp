#include "HornMethod.h"

HornMethod::HornMethod()
{
}

void print(CvMat *ok)
{
	for(int i = 0; i < ok->rows; i++)
	{
		for(int j = 0; j < ok->cols; j++)
			cout << cvmGet(ok, i,j) << " ";
		cout << endl;
	}
}

void HornMethod::getTransformation(vector<cv::Point3f> *setA, vector<cv::Point3f> *setB, Transformation *Result )
{
	int i, pairNum = setA->size();
	CvMat *muLmuR = cvCreateMat(3, 3, cv::DataType<double>::type);
	CvMat *M = cvCreateMat(3, 3, cv::DataType<double>::type);
	CvMat *curMat = cvCreateMat(3, 3, cv::DataType<double>::type);
	CvMat *N = cvCreateMat(4, 4, cv::DataType<double>::type);

	// INITALIZE MATRICES
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
		{
			cvmSet(muLmuR, i, j, 0);
			cvmSet(M, i, j, 0);
			cvmSet(curMat, i, j, 0);
		}

	for(int i = 0; i < 4; i++)
		for(int j = 0; j < 4; j++)
			cvmSet(N, i, j, 0);
		

	cv::Point3d meanA(0, 0, 0), meanB(0, 0, 0);

	for(int i = 0; i < pairNum; i++)
	{
		meanA.x += setA->at(i).x;
		meanA.y += setA->at(i).y;
		meanA.z += setA->at(i).z;

		meanB.x += setB->at(i).x;
		meanB.y += setB->at(i).y;
		meanB.z += setB->at(i).z;
	}
	
	meanA.x = meanA.x / pairNum;
	meanA.y = meanA.y / pairNum;
	meanA.z = meanA.z / pairNum;

	meanB.x = meanB.x / pairNum;
	meanB.y = meanB.y / pairNum;
	meanB.z = meanB.z / pairNum;
	
	cvmSet(muLmuR, 0,0, meanA.x * meanB.x);
	cvmSet(muLmuR, 0,1, meanA.x * meanB.y);
	cvmSet(muLmuR, 0,2, meanA.x * meanB.z);
	cvmSet(muLmuR, 1,0, meanA.y * meanB.x);
	cvmSet(muLmuR, 1,1, meanA.y * meanB.y);
	cvmSet(muLmuR, 1,2, meanA.y * meanB.z);
	cvmSet(muLmuR, 2,0, meanA.z * meanB.x);
	cvmSet(muLmuR, 2,1, meanA.z * meanB.y);
	cvmSet(muLmuR, 2,2, meanA.z * meanB.z);

		
	for (int i = 0; i < setA->size(); i++) {
		cv::Point3f leftPoint = setA->at(i);
		cv::Point3f rightPoint = setB->at(i);
	
		cvmSet(curMat, 0,0, leftPoint.x * rightPoint.x);
		cvmSet(curMat, 0,1, leftPoint.x * rightPoint.y);
		cvmSet(curMat, 0,2, leftPoint.x * rightPoint.z);
		cvmSet(curMat, 1,0, leftPoint.y * rightPoint.x);
		cvmSet(curMat, 1,1, leftPoint.y * rightPoint.y);
		cvmSet(curMat, 1,2, leftPoint.y * rightPoint.z);
		cvmSet(curMat, 2,0, leftPoint.z * rightPoint.x);
		cvmSet(curMat, 2,1, leftPoint.z * rightPoint.y);
		cvmSet(curMat, 2,2, leftPoint.z * rightPoint.z);
		cvmAdd(M, curMat, M);
	}

	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
		{
			double tmp =  -1 * pairNum * cvmGet(muLmuR, i, j);
			cvmSet(muLmuR, i, j ,  tmp);
		}
	
	cvmAdd(M, muLmuR ,M);
	

   	//compute the matrix N	
	CvMat *tmp = cvCreateMat(3, 3, cv::DataType<double>::type);
	for(int i = 0; i < 3; i++)
	  for(int j = 0; j < 3; j++)
		  cvmSet(tmp, i, j, 0);

    double traceM = 0.0;
    for(int i = 0; i < 3; i++)
      traceM += cvmGet(M,i,i);
   
    cvmSet(tmp, 0, 0, -1 * traceM);
    cvmSet(tmp, 1, 1, -1 * traceM);
    cvmSet(tmp, 2, 2, -1 * traceM);

    CvMat *MTranspose = cvCreateMat(3, 3, cv::DataType<double>::type);
    cvTranspose(M, MTranspose);
 
    cvmAdd(tmp, MTranspose, tmp); 
    cvmAdd(tmp, M, tmp); 
  
  

    double A12, A20, A01;
    A12 = cvmGet(M, 1, 2) - cvmGet(M, 2, 1);
    A20 = cvmGet(M, 2, 0) - cvmGet(M, 0, 2);
    A01 = cvmGet(M, 0, 1) - cvmGet(M, 1, 0);
 
    cvmSet(N, 0, 0, traceM);
    cvmSet(N, 0, 1, A12);
    cvmSet(N, 0, 2, A20);
    cvmSet(N, 0, 3, A01);

    cvmSet(N, 1, 0, A12);
    cvmSet(N, 2, 0, A20);
    cvmSet(N, 3, 0, A01);

  for(int i = 1; i < 4; i++)
	  for(int j = 1; j < 4; j++)
		cvmSet(N, i, j, cvmGet(tmp, i - 1, j - 1));
            
  CvMat* eigenVec = cvCreateMat(4, 4, cv::DataType<double>::type);
  CvMat* eigenVal  =cvCreateMat(4, 1, cv::DataType<double>::type);
  cvEigenVV(N, eigenVec, eigenVal);
  Transformation transform(false);
  transform.setRotation( -1 * cvmGet(eigenVec, 0, 0), -1 * cvmGet(eigenVec, 0, 1), -1 * cvmGet(eigenVec, 0, 2), -1 * cvmGet(eigenVec, 0, 3), true );
  
  transform.applyToPoint(&meanA);
  
  Result->setRotation( -1 * cvmGet(eigenVec, 0, 0), -1 * cvmGet(eigenVec, 0, 1), -1 * cvmGet(eigenVec, 0, 2), -1 * cvmGet(eigenVec, 0, 3), true );
  Result->setTranslation( meanB.x - meanA.x, meanB.y - meanA.y, meanB.z - meanA.z );
 
}


void HornMethod::selfTest()
{
 //   int i, pairNum = 10;
	//double bounds = 100; //point coordinates are in [-100,100]
	//double maxTranslation = 1000; //translations are in [-1000,1000]
	//std::vector<Point3f *> left, right;
 //   double noiseSpreadRadius = 3; //noise maximal magnitude
	//Point3f leftPoint, rightPoint;
 //   Transformation knownTransformation, invKnownTransformation, computedTransformation, diffTransformation;

 //           //1. Create a random transformation and a random point set.
	//          //   Transform the point set to get the second point set.

	//srand((unsigned)time(NULL)); //seed random number generator

	//                          //create a random transformation
	//double tx = maxTranslation*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//double ty = maxTranslation*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//double tz = maxTranslation*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//            //get random unit quaternion
	//double s = (double)rand();
	//double qx = (double)rand();
	//double qy = (double)rand();
	//double qz = (double)rand();
	//double norm = sqrt(s*s + qx*qx + qy*qy + qz*qz);  
	//s/=norm;	
	//qx/=norm;	
	//qy/=norm;
	//qz/=norm;
 //          
 // knownTransformation.setRotation(s,qx,qy,qz);
 // knownTransformation.setTranslation(tx,ty,tz);


 //                     //create random points, transform them, and add noise
	//for(i=0; i<pairNum; i++) {
	//	Point3f * leftPoint = new Point3f(0, 0, 0);
	//	Point3f * rightPoint = new Point3f(0, 0, 0);
	//	leftPoint->x = bounds*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//	leftPoint->y = bounds*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//	leftPoint->z = bounds*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//	knownTransformation.apply(leftPoint, rightPoint);
	//	rightPoint->x += noiseSpreadRadius*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//	rightPoint->y += noiseSpreadRadius*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//	rightPoint->z += noiseSpreadRadius*(double)rand()/(double)RAND_MAX * (rand() > RAND_MAX/2 ? 1 : -1);
	//	left.push_back(leftPoint);
 //   right.push_back(rightPoint);
	//}
	//          //estimate transformation
 // this->getTransformation(left,right,&computedTransformation);

 //          //how does known^-1*computed, differ from the identity
 // double parameterDiff[6];  
 // bool isGimbalLocked;
 // invKnownTransformation.invert(&knownTransformation);
 // diffTransformation.mul(&invKnownTransformation,&computedTransformation);
 // //diffTransformation.getRotationAngles(parameterDiff, isGimbalLocked); //I'm not checking if it is gimbal locked
 // //diffTransformation.getTranslation(&(parameterDiff[3]));

 // cout<<"Known transformation:\n";
 // cout<<"*********************\n";
 // for(int i = 0; i < 3; i++)
	//  for(int j = 0; j < 3; j++)
	//	  cout<< knownTransformation.rotation[i][j]<<"\n";

 // 

 // cout<<"Computed transformation:\n";
 // cout<<"***********************\n";
 // for(int i = 0; i < 3; i++)
	//  for(int j = 0; j < 3; j++)
	//	  cout<< computedTransformation.rotation[i][j]<<"\n";

 /* cout<<"known^-1*computed, difference from identity [theta_x,theta_y,theta_z,t_x,t_y,t_z] (angles in degrees):\n";
  cout<<"******************************************************************************************************\n\t";
  cout<<" ["<<Frame::toDegrees(parameterDiff[0])<<", "<<Frame::toDegrees(parameterDiff[1])<<", "<<Frame::toDegrees(parameterDiff[2])<<", ";
  cout<<parameterDiff[3]<<", "<<parameterDiff[4]<<", "<<parameterDiff[5]<<"]\n";*/
}

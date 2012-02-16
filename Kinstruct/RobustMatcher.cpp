#include "RobustMatcher.h"

	RobustMatcher::RobustMatcher() : ratio(0.80f), refineF(true), confidence(0.1), distance(1) {
	}


	int RobustMatcher::track(cv::Mat& image1,	cv::Mat& image2) {
		std::vector<cv::Point2f> selPoints2;
		std::vector<uchar> status;
		std::vector<float> err;
		std::vector<cv::Point2f> features;
		cv::goodFeaturesToTrack(image1, // the image
		initial, // the output detected features
		500, // the maximum number of features
		0.001, // quality level
		5, cv::Mat(), 3, 0, 0.04); // min distance between two features



		cv::calcOpticalFlowPyrLK(
			image1, image2, // 2 consecutive images
			initial, // input point positions in first image
			selPoints2, // output point positions in the 2nd image
			status, // tracking success
			err, cv::Size(5,5), 5, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ));
		

		int k=0;
		
		std::vector<uchar> inliers(initial.size(), 0);
		cv::Mat fundemental= cv::findFundamentalMat(
		cv::Mat(initial),cv::Mat(selPoints2), // matching points
		inliers, // match status (inlier or outlier)
		CV_FM_RANSAC, // RANSAC method
		distance, // distance to epipolar line
		confidence); // confidence probability
		// extract the surviving (inliers) matches
		std::vector<uchar>::const_iterator	itIn= inliers.begin();
		// for all matches
		int i = 0;
		for ( ;itIn!= inliers.end(); ++itIn, ++i) {
			if (*itIn && status[i]) { // it is a valid match

				initial[k]= initial[i];
					final.push_back(selPoints2[i]);
					k++;
			}
		}
		

		final.resize(k);
		initial.resize(k);
		cout << "correct tracking: " << k << endl;

		for(int i= 0; i < final.size(); i++ ) {
		// draw line and circle
			cv::line(image2,
			initial[i], // initial position
			final[i],// new position
			cv::Scalar(255,0,0));
			cv::circle(image2, final[i], 3,
			cv::Scalar(0,255,0),-1);
		}
		return k;

	}

		
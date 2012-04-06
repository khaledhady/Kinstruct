#include "Commons.h"

#ifndef TRACKER
#define TRACKER

using namespace std;


class Tracker {
	private:

		double distance; // min distance to epipolar
		double confidence; // confidence level (probability)

	public:
		Tracker();
		std::vector<cv::Point2f> initial;
		std::vector<cv::Point2f> final;
		double track(cv::Mat& image1,	cv::Mat& image2);
		cv::Mat Tracker::match(cv::Mat& image1,
	cv::Mat& image2, // input images
	// output matches and keypoints
	std::vector<cv::DMatch>& matches,
	std::vector<cv::KeyPoint>& keypoints1,
	std::vector<cv::KeyPoint>& keypoints2);
};

#endif
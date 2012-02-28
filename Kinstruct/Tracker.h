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
		int track(cv::Mat& image1,	cv::Mat& image2);
};

#endif
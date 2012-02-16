#ifndef COMMONS
#define COMMONS
#include "Commons.h"
#endif

#ifndef MATCHER
#define MATCHER

using namespace std;

/* Using the GLUT library for the base windowing setup */
#include <GL/glut.h>
#include <GL/gl.h>
#include <math.h>

class RobustMatcher {


	private:

		// pointer to the feature point detector object
		cv::Ptr<cv::FeatureDetector> detector;
		// pointer to the feature descriptor extractor object
		cv::Ptr<cv::DescriptorExtractor> extractor;
		float ratio; // max ratio between 1st and 2nd NN
		bool refineF; // if true will refine the F matrix
		double distance; // min distance to epipolar
		double confidence; // confidence level (probability)

	public:

		RobustMatcher();
		std::vector<cv::Point2f> initial;
		std::vector<cv::Point2f> final;
		int track(cv::Mat& image1,	cv::Mat& image2);
};

#endif
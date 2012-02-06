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

		/*Estimating Projective Relations in Images
		Note how we used the generic cv::FeatureDetector and cv::DescriptorExtractor
		interfaces so that a user can provide any specific implementation. The SURF features and
		descriptors are used here by default, but others can be specified using the appropriate setter
		methods:*/

		// Set the feature detector
		void setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect);

		// Set the descriptor extractor
		void setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor>& desc);

		//The main method is our match method that returns matches, detected keypoints, and the
		//estimated fundamental matrix. The method proceeds in five distinct steps (explicitly identified
		//in the comments of the following code) that we will now explore:
		// Match feature points using symmetry test and RANSAC
		// returns fundemental matrix
		cv::Mat match(cv::Mat& image1,	cv::Mat& image2, std::vector<cv::DMatch>& matches,
					std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2);

		//The first step is simply detecting the feature point and computing their descriptors. Next, we
		//proceed to feature matching using the cv::BruteForceMatcher class as we did in the
		//previous chapter. However, this time we find the two best matching points for each feature
		//(and not only the best one as we did in the previous recipe). This is accomplished by the
		//cv::BruteForceMatcher::knnMatch method (with k=2). Moreover, we perform this
		//matching in two directions, that is, for each point in the first image we find the two best
		//matches in the second image, and then we do the same thing for the feature points of the
		//second image, finding their two best matches in the first image.
		//Therefore, for each feature point, we have two candidate matches in the other view. These are
		//the two best ones based on the distance between their descriptors. If this measured distance
		//is very low for the best match, and much larger for the second best match, we can safely
		//accept the first match as a good one since it is unambiguously the best choice. Reciprocally,
		//if the two best matches are relatively close in distance, then there exists a possibility that we
		//make an error if we select one or the other. In this case, we should reject both matches. Here,
		//we perform this test in step 3 by verifying that the ratio of the distance of the best match over
		//the distance of the second best match is not greater than a given threshold:
		//Estimating Projective Relations in Images

		// Clear matches for which NN ratio is > than threshold
		// return the number of removed points
		// (corresponding entries being cleared,
		// i.e. size will be 0)
		int ratioTest(std::vector<std::vector<cv::DMatch>>	&matches);

		//A large number of ambiguous matches will be eliminated by this procedure as it can be seen
		//from the following example. Here, with a low SURF threshold (=10), we initially detected
		//1,600 feature points (black circles) out of which only 55 survive the ratio test (white circles):
		//The white lines linking the matched points show that even if we have a large number of good
		//matches, a significant number of false matches have survived. Therefore, a second test will
		//be performed in order to filter our more false matches. Note that the ratio test is also applied
		//to the second match set.
		//We now have two relatively good match sets, one from the first image to second image and the
		//other one from second image to the first one. From these sets, we will now extract the matches
		//that are in agreement with both sets. This is the symmetrical matching scheme imposing that,
		//for a match pair to be accepted, both points must be the best matching feature of the other:
		// Insert symmetrical matches in symMatches vector
		void symmetryTest(const std::vector<std::vector<cv::DMatch>>& matches1,
						  const std::vector<std::vector<cv::DMatch>>& matches2,
						  std::vector<cv::DMatch>& symMatches);

		//Estimating Projective Relations in Images
		//In our test pair, 31 matches survived this symmetry test. The last test now consists of an
		//additional filtering test that will this time use the fundamental matrix in order to reject
		//matches that do not obey the epipolar constraint. This test is based on the RANSAC method
		//that can compute the fundamental matrix even when outliers are still present in the match set
		//(this method will be explained in the following section):
		// Identify good matches using RANSAC
		// Return fundemental matrix
		cv::Mat ransacTest(const std::vector<cv::DMatch>& matches,
						   const std::vector<cv::KeyPoint>& keypoints1,
						   const std::vector<cv::KeyPoint>& keypoints2,
						   std::vector<cv::DMatch>& outMatches);
};

#endif
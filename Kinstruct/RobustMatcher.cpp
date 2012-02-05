#include "RobustMatcher.h"

	RobustMatcher::RobustMatcher() : ratio(0.65f), refineF(true), confidence(0.1), distance(1) {
		// SURF is the default feature
		detector= new cv::SurfFeatureDetector();
		extractor= new cv::SurfDescriptorExtractor();
	}

	/*Estimating Projective Relations in Images
	Note how we used the generic cv::FeatureDetector and cv::DescriptorExtractor
	interfaces so that a user can provide any specific implementation. The SURF features and
	descriptors are used here by default, but others can be specified using the appropriate setter
	methods:*/
	// Set the feature detector
	void RobustMatcher::setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect) {
		detector= detect;
	}
	// Set the descriptor extractor
	void RobustMatcher::setDescriptorExtractor(
		cv::Ptr<cv::DescriptorExtractor>& desc) {
		extractor= desc;
	}
	//The main method is our match method that returns matches, detected keypoints, and the
	//estimated fundamental matrix. The method proceeds in five distinct steps (explicitly identified
	//in the comments of the following code) that we will now explore:
	// Match feature points using symmetry test and RANSAC
	// returns fundemental matrix
	cv::Mat RobustMatcher::match(cv::Mat& image1,	cv::Mat& image2, 
				std::vector<cv::DMatch>& matches,
				std::vector<cv::KeyPoint>& keypoints1,
				std::vector<cv::KeyPoint>& keypoints2) {
		// 1a. Detection of the SURF features
		detector->detect(image1,keypoints1);
		detector->detect(image2,keypoints2);
		// 1b. Extraction of the SURF descriptors
		cv::Mat descriptors1, descriptors2;
		extractor->compute(image1,keypoints1,descriptors1);
		extractor->compute(image2,keypoints2,descriptors2);
		// 2. Match the two image descriptors
		// Construction of the matcher
		cv::BruteForceMatcher<cv::L2<float>> matcher;
		// from image 1 to image 2
		// based on k nearest neighbours (with k=2)
		std::vector<std::vector<cv::DMatch>> matches1;
		matcher.knnMatch(descriptors1,descriptors2,
		matches1, // vector of matches (up to 2 per entry)
		2); // return 2 nearest neighbours
		// from image 2 to image 1
		// based on k nearest neighbours (with k=2)
		std::vector<std::vector<cv::DMatch>> matches2;
		matcher.knnMatch(descriptors2,descriptors1,
		matches2, // vector of matches (up to 2 per entry)
		2); // return 2 nearest neighbours
		// 3. Remove matches for which NN ratio is
		// > than threshold
		// clean image 1 -> image 2 matches
		//int removed = ratioTest(matches1);
		//// clean image 2 -> image 1 matches
		//removed= ratioTest(matches2);
		// 4. Remove non-symmetrical matches
		std::vector<cv::DMatch> symMatches;
		symmetryTest(matches1,matches2,symMatches);
		// 5. Validate matches using RANSAC
		cv::Mat fundemental = ransacTest(symMatches,
		keypoints1, keypoints2, matches);
		// return the found fundemental matrix
		return fundemental;
	}
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
	int RobustMatcher::ratioTest(std::vector<std::vector<cv::DMatch>>	&matches) {
		int removed=0;
		// for all matches
		for (std::vector<std::vector<cv::DMatch>>::iterator	matchIterator= matches.begin();	matchIterator!= matches.end(); ++matchIterator) {
			// if 2 NN has been identified
			if (matchIterator->size() > 1) {
				// check distance ratio
				if ((*matchIterator)[0].distance/(*matchIterator)[1].distance > ratio) {
					matchIterator->clear(); // remove match
					removed++;
				}
			} else { // does not have 2 neighbours
				matchIterator->clear(); // remove match
				removed++;
			}
		}
		return removed;
	}
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
	void RobustMatcher::symmetryTest(
	const std::vector<std::vector<cv::DMatch>>& matches1,
	const std::vector<std::vector<cv::DMatch>>& matches2,
	std::vector<cv::DMatch>& symMatches) {
	// for all matches image 1 -> image 2
	for (std::vector<std::vector<cv::DMatch>>::
	const_iterator matchIterator1= matches1.begin();
	matchIterator1!= matches1.end(); ++matchIterator1) {
	// ignore deleted matches
	if (matchIterator1->size() < 2)
	continue;
	// for all matches image 2 -> image 1
	for (std::vector<std::vector<cv::DMatch>>::
	const_iterator matchIterator2= matches2.begin();
	matchIterator2!= matches2.end();
	++matchIterator2) {
	// ignore deleted matches
	if (matchIterator2->size() < 2)
	continue;
	// Match symmetry test
	if ((*matchIterator1)[0].queryIdx ==
	(*matchIterator2)[0].trainIdx &&
	(*matchIterator2)[0].queryIdx ==
	(*matchIterator1)[0].trainIdx) {
	// add symmetrical match
	symMatches.push_back(
	cv::DMatch((*matchIterator1)[0].queryIdx,
	(*matchIterator1)[0].trainIdx,
	(*matchIterator1)[0].distance));
	break; // next match in image 1 -> image 2
	}
	}
	}
	}
	//Estimating Projective Relations in Images
	//In our test pair, 31 matches survived this symmetry test. The last test now consists of an
	//additional filtering test that will this time use the fundamental matrix in order to reject
	//matches that do not obey the epipolar constraint. This test is based on the RANSAC method
	//that can compute the fundamental matrix even when outliers are still present in the match set
	//(this method will be explained in the following section):
	// Identify good matches using RANSAC
	// Return fundemental matrix
	cv::Mat RobustMatcher::ransacTest(
	const std::vector<cv::DMatch>& matches,
	const std::vector<cv::KeyPoint>& keypoints1,
	const std::vector<cv::KeyPoint>& keypoints2,
	std::vector<cv::DMatch>& outMatches) {
	// Convert keypoints into Point2f
	std::vector<cv::Point2f> points1, points2;
	for (std::vector<cv::DMatch>::
	const_iterator it= matches.begin();
	it!= matches.end(); ++it) {
	// Get the position of left keypoints
	float x= keypoints1[it->queryIdx].pt.x;
	float y= keypoints1[it->queryIdx].pt.y;
	points1.push_back(cv::Point2f(x,y));
	// Get the position of right keypoints
	x= keypoints2[it->trainIdx].pt.x;
	y= keypoints2[it->trainIdx].pt.y;
	points2.push_back(cv::Point2f(x,y));
	}
	// Compute F matrix using RANSAC
	std::vector<uchar> inliers(points1.size(),0);
	cv::Mat fundemental= cv::findFundamentalMat(
	cv::Mat(points1),cv::Mat(points2), // matching points
	inliers, // match status (inlier or outlier)
	CV_FM_RANSAC, // RANSAC method
	distance, // distance to epipolar line
	confidence); // confidence probability
	// extract the surviving (inliers) matches
	std::vector<uchar>::const_iterator
	itIn= inliers.begin();
	std::vector<cv::DMatch>::const_iterator
	itM= matches.begin();
	// for all matches
	for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
	if (*itIn) { // it is a valid match

	outMatches.push_back(*itM);
	}
	}
	if (refineF) {
	// The F matrix will be recomputed with
	// all accepted matches
	// Convert keypoints into Point2f
	// for final F computation
	points1.clear();
	points2.clear();
	for (std::vector<cv::DMatch>::
	const_iterator it= outMatches.begin();
	it!= outMatches.end(); ++it) {
	// Get the position of left keypoints
	float x= keypoints1[it->queryIdx].pt.x;
	float y= keypoints1[it->queryIdx].pt.y;
	points1.push_back(cv::Point2f(x,y));
	// Get the position of right keypoints
	x= keypoints2[it->trainIdx].pt.x;
	y= keypoints2[it->trainIdx].pt.y;
	points2.push_back(cv::Point2f(x,y));
	}
	// Compute 8-point F from all accepted matches
	//fundemental= cv::findFundamentalMat(
	//cv::Mat(points1),cv::Mat(points2), // matches
	//CV_FM_8POINT); // 8-point method
	}
	return fundemental;
	}
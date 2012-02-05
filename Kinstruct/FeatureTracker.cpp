#include <opencv/cv.h>
#include <opencv/cxcore.h>


class FeatureTracker  {
	public:
	cv::Mat grayA;
	cv::Mat grayB;
	// tracked features from 0->1
	std::vector<cv::Point2f> points[2];
	// initial position of tracked points
	std::vector<cv::Point2f> initial;
	std::vector<cv::Point2f> features; // detected features
	int max_count; // maximum number of features to detect
	double qlevel; // quality level for feature detection
	double minDist; // min distance between two points
	std::vector<uchar> status; // status of tracked features
	std::vector<float> err; // error in tracking

	

		void track(cv:: Mat &colorA, cv:: Mat &colorB)
		{
			cv::cvtColor(colorA, grayA, CV_BGR2GRAY);
			cv::cvtColor(colorB, grayB, CV_BGR2GRAY);
			// 1. if new feature points must be added
			if(addNewPoints())
			{
				// detect feature points
				detectFeaturePoints();
				// add the detected features to
				// the currently tracked features
				points[0].insert(points[0].end(),
				features.begin(),features.end());
				initial.insert(initial.end(),
				features.begin(),features.end());
			}
			// 2. track features
			cv::calcOpticalFlowPyrLK(
			grayA, grayB, // 2 consecutive images
			points[0], // input point positions in first image
			points[1], // output point positions in the 2nd image
			status, // tracking success
			err); // tracking error
			// 2. loop over the tracked points to reject some
			int k=0;
			for( int i= 0; i < points[1].size(); i++ ) {
				// do we keep this point?
				if (acceptTrackedPoint(i)) {
					// keep this point in vector
					initial[k]= initial[i];
					points[1][k++] = points[1][i];
				}
			}
			// eliminate unsuccesful points
			points[1].resize(k);
			initial.resize(k);
			handleTrackedPoints(colorA, colorB);
		}
	//Next, we define the process method that will be called for each frame of the sequence.
	//Basically, we need to proceed as follows. First, feature points are detected if necessary. Next,
	//these points are tracked. You reject points that you cannot track or you no longer want to track.
	//You are now ready to handle the successfully tracked points. Finally, the current frame and its
	//points become the previous frame and points for the next iteration. Here is how to do it:
	//void process(cv:: Mat &frame, cv:: Mat &output) {
	//// convert to gray-level image
	//	cv::cvtColor(frame, gray, CV_BGR2GRAY);
	//	frame.copyTo(output);
	//	// 1. if new feature points must be added
	//	if(addNewPoints())
	//	{
	//		// detect feature points
	//		detectFeaturePoints();
	//		// add the detected features to
	//		// the currently tracked features
	//		points[0].insert(points[0].end(),
	//		features.begin(),features.end());
	//		initial.insert(initial.end(),
	//		features.begin(),features.end());
	//	}
	//	// for first image of the sequence
	//	if(gray_prev.empty())
	//		gray.copyTo(gray_prev);
	//	// 2. track features
	//	cv::calcOpticalFlowPyrLK(
	//	gray_prev, gray, // 2 consecutive images
	//	points[0], // input point positions in first image
	//	points[1], // output point positions in the 2nd image
	//	status, // tracking success
	//	err); // tracking error
	//	// 2. loop over the tracked points to reject some
	//	int k=0;
	//	for( int i= 0; i < points[1].size(); i++ ) {
	//		// do we keep this point?
	//		if (acceptTrackedPoint(i)) {
	//			// keep this point in vector
	//			initial[k]= initial[i];
	//			points[1][k++] = points[1][i];
	//		}
	//	}

	//	// eliminate unsuccesful points
	//	points[1].resize(k);
	//	initial.resize(k);
	//	// 3. handle the accepted tracked points
	//	handleTrackedPoints(frame, output);
	//	// 4. current points and image become previous ones
	//	std::swap(points[1], points[0]);
	//	cv::swap(gray_prev, gray);
	//}
/*his method makes use of four other utility methods. It should be easy for you to
change any of these methods in order to define a new behavior for your own tracker. The
first of these methods detects the feature points. Note that we already discussed the
cv::goodFeatureToTrack function in the first recipe of Chapter 8:*/
// feature point detection
void detectFeaturePoints() {
// detect the features
cv::goodFeaturesToTrack(grayA, // the image
features, // the output detected features
500, // the maximum number of features
0.01, // quality level
10); // min distance between two features
}
//The second one determines if new feature points should be detected:
// determine if new points should be added
bool addNewPoints() {
// if too few points
return points[0].size()<=10;
}
//The third one rejects some of the tracked points based on some criteria defined by the
//application. Here, we decided to reject points that do not move (in addition to those that
//cannot be tracked by the cv::calcOpticalFlowPyrLK function):
// determine which tracked point should be accepted
bool acceptTrackedPoint(int i) {
return status[i];
}
//
//Finally, the fourth method handles the tracked feature points by drawing on the current frame
//all of the tracked points with a line joining them to their initial position (that is, the position
//where they were detected the first time):
// handle the currently tracked points
	void handleTrackedPoints(cv:: Mat &frame,
	cv:: Mat &output) {
	// for all tracked points
		for(int i= 0; i < points[1].size(); i++ ) {
		// draw line and circle
			cv::line(output,
			initial[i], // initial position
			points[1][i],// new position
			cv::Scalar(255,255,255));
			cv::circle(output, points[1][i], 3,
			cv::Scalar(255,255,255),-1);
		}
	}
};
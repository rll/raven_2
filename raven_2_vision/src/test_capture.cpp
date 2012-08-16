#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <LinearMath/btTransform.h>
#include <ros/ros.h>
#include "raven_2_vision/raven_vision.h"

using namespace std;
using namespace cv;
#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)


int main(int argc, char* argv[]) {
	const char* calibFile = EXPAND(RAVEN_VISION_DATA_DIR)"/logitech_intrinsic_params.yaml";
	cout << calibFile << endl;
	FileStorage fs2;
	fs2.open(calibFile, FileStorage::READ);

	cout << fs2.isOpened() << endl;
	// first method: use (type) operator on FileNode.
	Mat cameraMatrix, distCoeffs;
	fs2["camera_matrix"] >> cameraMatrix;
	fs2["distortion_coefficients"] >> distCoeffs;
	cout << cameraMatrix << endl;
	cout << distCoeffs << endl;


	VideoCapture capture;
	capture.open(0);


	const char* windowName = "Image View";
	namedWindow(windowName, 1 );
	ros::init(argc, argv,"test_capture");
	ros::NodeHandle nh;
	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("camera_pose",10);

	while (ros::ok()) {
		cv::Mat image0, image;
		capture >> image0;
		if (image0.data == NULL) throw runtime_error("no image");
		image0.copyTo(image);
		imshow(windowName, image);
		int key = waitKey(10);
		geometry_msgs::PoseStamped ps;
		bool gotPose = getChessboardPoseNoRect(image, Size(6,8), .0157, cameraMatrix, distCoeffs, ps.pose);
		if (gotPose) {
			ps.header.frame_id = "/logitech0";
			ps.header.stamp = ros::Time::now();
			posePub.publish(ps);
			cout << ps << endl;
		}
		else cout << "didn't find chessboard" << endl;
	}
}

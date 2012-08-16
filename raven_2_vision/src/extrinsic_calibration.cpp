#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <LinearMath/btTransform.h>
#include "raven_2_vision/raven_vision.h"

#include "config.h"

using namespace std;
using namespace cv;
#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

struct LocalConfig : Config {
	static int camID;
	static int width;
	static int height;
	LocalConfig() : Config() {
		params.push_back(new Parameter<int>("camID", &camID, "camera id"));
		params.push_back(new Parameter<int>("width", &width, "chessboard width"));
		params.push_back(new Parameter<int>("height", &height, "chessboard height"));
	}
};
int LocalConfig::camID = 0;
int LocalConfig::width = 6;
int LocalConfig::height = 8;

int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	const char* intrinsicsFile = EXPAND(RAVEN_VISION_DATA_DIR)"/logitech_intrinsics.yaml";
	cout << "getting camera parameters from " << intrinsicsFile << endl;
	FileStorage intrinsicsStorage;
	intrinsicsStorage.open(intrinsicsFile, FileStorage::READ);
	assert(intrinsicsStorage.isOpened());


	// first method: use (type) operator on FileNode.
	Mat cameraMatrix, distCoeffs;
	intrinsicsStorage["camera_matrix"] >> cameraMatrix;
	intrinsicsStorage["distortion_coefficients"] >> distCoeffs;


	string extrinsicsFile = (boost::format(EXPAND(RAVEN_VISION_DATA_DIR)"/extrinsics%i.txt")%LocalConfig::camID).str();
	cout << extrinsicsFile << endl;


	VideoCapture capture;
	capture.open(LocalConfig::camID);
	if (!capture.isOpened()) throw runtime_error("could not open camera");

	const char* windowName = "Image View";
	namedWindow(windowName, 1 );

	geometry_msgs::Pose pose;
	bool poseWasFound = false;
	while (true) {
		cv::Mat image0, image;
		capture >> image0;
		if (image0.data == NULL) throw runtime_error("no image");
		image0.copyTo(image);

		bool gotPose = getChessboardPoseNoRect(image, Size(LocalConfig::width, LocalConfig::height), .0157, cameraMatrix, distCoeffs, pose);
		if (gotPose) {
			poseWasFound = true;
			cout << "found checkerboard. press q to quit" << endl;
		}


		imshow(windowName, image);
		int key = waitKey(10);
		if (key == 'q') break;


	}

	if (poseWasFound) {
		cout << "writing position + rotation to " << extrinsicsFile << endl;
		ofstream extrinsicsStorage(extrinsicsFile.c_str());
		if (extrinsicsStorage.bad()) throw runtime_error("couldn't open output file");
		extrinsicsStorage << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << endl;
	}



}

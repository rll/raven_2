#include "raven_2_vision/raven_vision.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <LinearMath/btTransform.h>
#include <ros/ros.h>
using namespace std;
using namespace cv;




vector<Point3f> calcChessboardCorners(Size boardSize, float squareSize) {
	vector<Point3f> corners;
	float halfHeight = (boardSize.height-1.)/2;
	float halfWidth = (boardSize.width-1.)/2;
	float offsetW = squareSize*(boardSize.width-1.)/2;
	for( int i = 0; i < boardSize.height; i++ )
	  for( int j = boardSize.width-1; j > -1; j--)
	    corners.push_back(Point3f((j-halfWidth)*squareSize, (i-halfHeight)*squareSize, 0));
	return corners;
}


geometry_msgs::Pose rodToPose(const cv::Mat& rvec, const cv::Mat& tvec) {
	cv::Mat rot0(3,3, CV_32F);
	Rodrigues(rvec, rot0);
	cv::Mat_<float> rot;
	rot0.copyTo(rot);
	btMatrix3x3 basis(btMatrix3x3(rot(0,0), rot(0,1), rot(0,2),
			rot(1,0), rot(1,1), rot(1,2),
			rot(2,0), rot(2,1), rot(2,2)));
	btQuaternion q;
	basis.getRotation(q);

	cv::Mat_<float> tra;
	tvec.copyTo(tra);
	btVector3 t(tra(0), tra(1), tra(2));

	geometry_msgs::Pose pose;
	pose.position.x = t.x();
	pose.position.y = t.y();
	pose.position.z = t.z();
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();

	return pose;
}

bool getChessboardPoseNoRect(Mat& image, Size boardSize, float squareSize, const Mat& cameraMatrix, const Mat& distCoeffs, geometry_msgs::Pose& poseOut, bool drawCorners,const char* windowName) {
	vector<Point3f> cornersBoard = calcChessboardCorners(boardSize, squareSize);
//	for (int i=0; i < cornersBoard.size(); i++) cout << i << " " << cornersBoard[i] << endl;
	vector<Point2f> cornersImage;
	bool boardFound = findChessboardCorners(image, boardSize, cornersImage,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK |CV_CALIB_CB_NORMALIZE_IMAGE);
	if (boardFound) {
//		for (int i=0; i < cornersImage.size(); i++) cout << i << " " << cornersImage[i] << endl;
//		cout << cameraMatrix << endl;
		Mat rvec, tvec;
		solvePnP(cornersBoard, cornersImage, cameraMatrix, distCoeffs, rvec, tvec, false);
		poseOut = rodToPose(rvec, tvec);
		if (drawCorners) {
			drawChessboardCorners(image, boardSize, cornersImage, true);
		}
	}
	if (windowName) {
		imshow(windowName, image);
	}

	return boardFound;

}

bool getChessboardPoseRect(Mat& image, Size boardSize, float squareSize, const Mat& cameraMatrix, const Mat& distCoeffs, geometry_msgs::Pose& poseOut, bool drawCorners,const char* windowName) {
	Mat image_rect;
	undistort(image,image_rect,cameraMatrix,distCoeffs);
	cv::Mat_<double> distCoeffs_rect(cv::Size(5,1), 0);
	return getChessboardPoseNoRect(image_rect,boardSize,squareSize,cameraMatrix,distCoeffs_rect,poseOut,drawCorners,windowName);

}

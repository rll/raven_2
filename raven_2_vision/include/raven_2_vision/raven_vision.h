#pragma once
#include <opencv2/core/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

std::vector<cv::Point3f> calcChessboardCorners(cv::Size boardSize, float squareSize);
geometry_msgs::Pose rodToPose(const cv::Mat& rvec, const cv::Mat& tvec);
bool getChessboardPoseNoRect(cv::Mat& image, cv::Size boardSize, float squareSize, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, geometry_msgs::Pose& poseOut, bool drawCorners=false,const char* windowName=0);
bool getChessboardPoseRect(cv::Mat& image, cv::Size boardSize, float squareSize, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, geometry_msgs::Pose& poseOut, bool drawCorners=false,const char* windowName=0);

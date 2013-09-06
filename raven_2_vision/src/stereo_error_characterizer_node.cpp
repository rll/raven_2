/*
 * Node to calculate the error in stereo estimation using a detected chessboard with a known location wrt to the camera.
 * The stereo pair should be mounted on the T-shaped calibration rig and the chessboard MUST BE PARALLEL TO THE XY PLANE OF THE CAMERA
 * The calibration rig ensures this. If this does not, hold, the measurements will be off
 * Uses raven_vision.cpp to find the chessboards and uses the point disparity to estimate the 3D location
 *
 * @author Jeff Mahler
 */
#include <iostream>
#include <fstream>
#include <time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_geometry/stereo_camera_model.h>
#include <LinearMath/btTransform.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/topic.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "config.h"
#include "raven_2_vision/position_error_table.hpp"
#include "raven_2_vision/raven_vision.h"

#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

struct LocalConfig : Config {
  static int width;
  static int height;
  static float square;
  static std::string stereo_name;
  static std::string left_pose_topic;
  static std::string right_pose_topic;
  static std::string left_image_topic;
  static std::string right_image_topic;
  static std::string left_info_topic;
  static std::string right_info_topic;
  static bool rect;
  static float detection_interval;
  static float print_interval;
  static std::string left_frame_id;
  static std::string right_frame_id;

  static int ref_corner_coord_i;       // coordinate measured wrt 0,0 being the top right corner
  static int ref_corner_coord_j;       // coordinate measured wrt 0,0 being the top right corner
  static float ref_corner_cam_basis_x; // x-component of 3d coordinate for the reference corner (in the left camera basis) 
  static float ref_corner_cam_basis_y; // y-component of 3d coordinate for the reference corner (in the left camera basis)
  static float chessboard_depth;       // depth of chessboard wrt to left camera (board should be parallel to xy plane of camera)
  static std::string error_filename;

  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("width", &width, "chessboard width"));
    params.push_back(new Parameter<int>("height", &height, "chessboard height"));
    params.push_back(new Parameter<float>("square", &square, "chessboard sidelength"));
    params.push_back(new Parameter<std::string>("stereo_name", &stereo_name, "stereo name"));
    params.push_back(new Parameter<std::string>("left_pose", &left_pose_topic, "left pose topic"));
    params.push_back(new Parameter<std::string>("right_pose", &right_pose_topic, "right pose topic"));
    params.push_back(new Parameter<std::string>("left_image", &left_image_topic, "left image topic"));
    params.push_back(new Parameter<std::string>("right_image", &right_image_topic, "right image topic"));
    params.push_back(new Parameter<std::string>("left_camera_info", &left_info_topic, "left camera info topic"));
    params.push_back(new Parameter<std::string>("right_camera_info", &right_info_topic, "right camera info topic"));
    params.push_back(new Parameter<bool>("rect",&rect,"rectify_image"));
    params.push_back(new Parameter<float>("detection_interval", &detection_interval, "detection interval"));
    params.push_back(new Parameter<float>("print_interval", &print_interval, "print interval"));
    params.push_back(new Parameter<std::string>("left_frame", &left_frame_id, "left frame id"));
    params.push_back(new Parameter<std::string>("right_frame", &right_frame_id, "right frame id"));
    params.push_back(new Parameter<int>("ref_corner_coord_i", &ref_corner_coord_i, "reference corner i-coordinate wrt to the top-left corner"));
    params.push_back(new Parameter<int>("ref_corner_coord_j", &ref_corner_coord_j, "reference corner j-coordinate wrt to the top-left corner"));
    params.push_back(new Parameter<float>("ref_corner_cam_basis_x", &ref_corner_cam_basis_x, "reference corner x-component of 3d position wrt camera basis"));
    params.push_back(new Parameter<float>("ref_corner_cam_basis_y", &ref_corner_cam_basis_y, "reference corner y-component of 3d position wrt camera basis"));
    params.push_back(new Parameter<float>("chessboard_depth", &chessboard_depth, "depth of the chessboard wrt camera basis"));
    params.push_back(new Parameter<std::string>("error_filename", &error_filename, "name of XML file to save to"));
  }
};
// size of the small chessboard mounted on cardboard
int LocalConfig::width = 10;
int LocalConfig::height = 7;
float LocalConfig::square = .0122;

std::string LocalConfig::stereo_name = "AD";
std::string LocalConfig::left_pose_topic = "left_chessboard_pose";
std::string LocalConfig::right_pose_topic = "right_chessboard_pose";
std::string LocalConfig::left_image_topic = std::string(LocalConfig::stereo_name).append("/left/image_rect");
std::string LocalConfig::right_image_topic = std::string(LocalConfig::stereo_name).append("/right/image_rect");
std::string LocalConfig::left_info_topic = std::string(LocalConfig::stereo_name).append("/left/camera_info");
std::string LocalConfig::right_info_topic = std::string(LocalConfig::stereo_name).append("/right/camera_info");
std::string LocalConfig::left_frame_id = std::string("left_").append(LocalConfig::stereo_name);
std::string LocalConfig::right_frame_id = std::string("right_").append(LocalConfig::stereo_name);

bool LocalConfig::rect = false;
float LocalConfig::detection_interval = 0.1;
float LocalConfig::print_interval = 1;

int LocalConfig::ref_corner_coord_i = 0;
int LocalConfig::ref_corner_coord_j = 0;
float LocalConfig::ref_corner_cam_basis_x = 0.0f;
float LocalConfig::ref_corner_cam_basis_y = 0.0f;
float LocalConfig::chessboard_depth = 0.0f;
std::string LocalConfig::error_filename = "stereo_estimation_errors.xml";

static tf::TransformBroadcaster* br;
static tf::TransformListener* listener;
static ros::Publisher left_pose_pub;
static ros::Publisher right_pose_pub;
static geometry_msgs::PoseStamped left_ps;
static geometry_msgs::PoseStamped right_ps;

static cv::Mat left_camera_matrix;
static cv::Mat right_camera_matrix;
static cv::Mat left_dist_coeff;
static cv::Mat right_dist_coeff;
static std::string left_window_name;
static std::string right_window_name;
static image_geometry::StereoCameraModel stereo_model;
static std::vector<cv::Point3f> actual_corners;

static unsigned int num_samples;
static std::vector<cv::Point3f> rms_error;
static std::vector<cv::Point3f> avg_error;
static std::vector<cv::Mat> covariance;

void image_callback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg) {
  cv::Mat leftImage = cv_bridge::toCvCopy(left_msg)->image;
  cv::Mat rightImage = cv_bridge::toCvCopy(right_msg)->image;
  tf::Transform leftCamToChessboard;
  tf::Transform rightCamToChessboard;

  // Get the left camera pose and corners
  bool gotLeftPose;
  cv::vector<cv::Point2f> cornersLeftImage;
  if (LocalConfig::rect) {
    gotLeftPose = getChessboardPoseRect(leftImage, cv::Size(LocalConfig::width, LocalConfig::height), LocalConfig::square,
					left_camera_matrix, left_dist_coeff, left_ps.pose, cornersLeftImage, true, left_window_name.c_str());
  }
  else {
    gotLeftPose = getChessboardPoseNoRect(leftImage, cv::Size(LocalConfig::width, LocalConfig::height), LocalConfig::square,
					  left_camera_matrix, left_dist_coeff, left_ps.pose, cornersLeftImage, true, left_window_name.c_str());
  }
  if (gotLeftPose) {
    left_ps.header.frame_id = LocalConfig::left_frame_id;
    left_ps.header.stamp = ros::Time::now();
    left_pose_pub.publish(left_ps);

    tf::Quaternion leftOrientation(left_ps.pose.orientation.x, left_ps.pose.orientation.y, left_ps.pose.orientation.z, left_ps.pose.orientation.w);
    tf::Vector3 leftPosition(left_ps.pose.position.x, left_ps.pose.position.y, left_ps.pose.position.z);
    tf::Transform chessboardToLeftCam(leftOrientation, leftPosition);
    br->sendTransform(tf::StampedTransform(chessboardToLeftCam, ros::Time::now(), LocalConfig::left_frame_id, "chessboard"));
  }

  // Get the right camera pose and corners
  bool gotRightPose;
  cv::vector<cv::Point2f> cornersRightImage;
  if (LocalConfig::rect) {
    gotRightPose = getChessboardPoseRect(rightImage, cv::Size(LocalConfig::width, LocalConfig::height), LocalConfig::square,
  					 right_camera_matrix, right_dist_coeff, right_ps.pose, cornersRightImage, true, right_window_name.c_str());
  }
  else {
    gotRightPose = getChessboardPoseNoRect(rightImage, cv::Size(LocalConfig::width, LocalConfig::height), LocalConfig::square,
  					   right_camera_matrix, right_dist_coeff, right_ps.pose, cornersRightImage, true, right_window_name.c_str());
  }
  if (gotRightPose) {
    right_ps.header.frame_id = LocalConfig::right_frame_id;
    right_ps.header.stamp = ros::Time::now();
    right_pose_pub.publish(right_ps);

    tf::Quaternion rightOrientation(right_ps.pose.orientation.x, right_ps.pose.orientation.y, right_ps.pose.orientation.z, right_ps.pose.orientation.w);
    tf::Vector3 rightPosition(right_ps.pose.position.x, right_ps.pose.position.y, right_ps.pose.position.z);
    tf::Transform chessboardToRightCam(rightOrientation, rightPosition);
    br->sendTransform(tf::StampedTransform(chessboardToRightCam, ros::Time::now(), LocalConfig::right_frame_id, "chessboard"));
  }

  // Project the measured corners to 3d and calculate the error
  if (gotLeftPose && gotRightPose && cornersLeftImage.size() == cornersRightImage.size()) {
    std::vector<cv::Point3f> cornerErrors;
    num_samples++;
    ROS_INFO_STREAM("Measuring frame " << num_samples);

    for (unsigned int i = 0; i < cornersLeftImage.size(); i++) {
      cv::Point3d corner3d;
      cv::Point3d error;
      float disparity = fabs(cornersLeftImage[i].x - cornersRightImage[i].x);
      stereo_model.projectDisparityTo3d(cornersLeftImage[i], disparity, corner3d);

      if (i == 0) {
	ROS_INFO_STREAM("ref corner: " << corner3d);
	if (LocalConfig::ref_corner_cam_basis_x == 0.0) {
	  LocalConfig::ref_corner_cam_basis_x = corner3d.x;
	}
	if (LocalConfig::ref_corner_cam_basis_y == 0.0) {
	  LocalConfig::ref_corner_cam_basis_y = corner3d.y;
	}
	if (LocalConfig::chessboard_depth == 0.0) {
	  LocalConfig::chessboard_depth = corner3d.z;
	}
      }
 
      // transform corner from left camera basis to chessboard basis
      geometry_msgs::PointStamped camCorner, chessCorner;
      camCorner.header.frame_id = LocalConfig::left_frame_id;
      camCorner.header.stamp = ros::Time();
      camCorner.point.x = corner3d.x;
      camCorner.point.y = corner3d.y;
      camCorner.point.z = corner3d.z;
      listener->transformPoint("chessboard", camCorner, chessCorner);

      // calculate the error in the 3D estimate
      error.x = actual_corners[i].x - chessCorner.point.x;
      error.y = actual_corners[i].y - chessCorner.point.y;
      error.z = actual_corners[i].z - chessCorner.point.z;
      cornerErrors.push_back(error);

      // online calculation of RMS error
      rms_error[i].x = sqrt((error.x*error.x + (num_samples-1)*rms_error[i].x*rms_error[i].x) / num_samples);
      rms_error[i].y = sqrt((error.y*error.y + (num_samples-1)*rms_error[i].y*rms_error[i].y) / num_samples);
      rms_error[i].z = sqrt((error.z*error.z + (num_samples-1)*rms_error[i].z*rms_error[i].z) / num_samples);

      cv::Mat covMat(cv::Size(1, 3), CV_32FC1);
      cv::Mat covMatTranspose;
      if (num_samples == 1) {
	// for the first sample the average error is just the current error
	avg_error[i].x = error.x;
	avg_error[i].y = error.y;
	avg_error[i].z = error.z;

	// update of covariance is not necessary here since one sample trivially has covariance of zero
      }
      else {
	covMat.at<float>(0, 0) = error.x - avg_error[i].x;
	covMat.at<float>(1, 0) = error.y - avg_error[i].y;
	covMat.at<float>(2, 0) = error.z - avg_error[i].z;
	cv::transpose(covMat, covMatTranspose);

	// online sample covariance estimate (see en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm)
	covariance[i] = ((num_samples-2) / (num_samples-1)) * covariance[i] + covMat * covMatTranspose / num_samples;

	// online calculation of avg error performed after covariance bc online covariance estimate uses avg_error_(n-1)
	avg_error[i].x = ((num_samples-1)*avg_error[i].x + error.x) / num_samples;
	avg_error[i].y = ((num_samples-1)*avg_error[i].y + error.y) / num_samples;
	avg_error[i].z = ((num_samples-1)*avg_error[i].z + error.z) / num_samples;
      }    
    }

    // for (unsigned int i = 0; i < avg_error.size(); i++) { 
    //   ROS_INFO_STREAM("Avg error for corner " << i << ": " << avg_error[i].x << " " << avg_error[i].y << " " << avg_error[i].z);
    // }
  }
}

void SaveErrorXML() {
  cv::FileStorage fs(LocalConfig::error_filename, cv::FileStorage::WRITE);

  if (!fs.isOpened()) {
    ROS_ERROR_STREAM("Failed to open stereo error file!");
    return;
  }

  time_t rawtime;
  time(&rawtime);
  fs << "experimentDate" << asctime(localtime(&rawtime));
  fs << "numSamples" << (int)num_samples;
  fs << "depth" << LocalConfig::chessboard_depth;
  fs << "xRange" << LocalConfig::width*LocalConfig::square;
  fs << "yRange" << LocalConfig::height*LocalConfig::square;

  // write 3d positions
  fs << "xyPositions" << "[";
  for (unsigned int j = 0 ; j < LocalConfig::height; j++) {
    for (unsigned int i = 0; i < LocalConfig::width; i++) {
      unsigned int index = i + j*LocalConfig::width;
      float x = LocalConfig::ref_corner_cam_basis_x + i*LocalConfig::square;
      float y = LocalConfig::ref_corner_cam_basis_y + j*LocalConfig::square;
      cv::Mat data = (cv::Mat_<float>(2, 1) << x, y);
      fs << data;
    }
  }
  fs << "]";

  // write average errors (same order as above)
  fs << "avgErrors" << "[";
  for (unsigned int j = 0 ; j < LocalConfig::height; j++) {
    for (unsigned int i = 0; i < LocalConfig::width; i++) {
      unsigned int index = i + j*LocalConfig::width;
      cv::Mat data = (cv::Mat_<float>(3, 1) << avg_error[index].x, avg_error[index].y, avg_error[index].z);
      fs << data;
    }
  }
  fs << "]";

  // write covariances (same order as above)
  fs << "covariance" << "[";
  for (unsigned int j = 0 ; j < LocalConfig::height; j++) {
    for (unsigned int i = 0; i < LocalConfig::width; i++) {
      unsigned int index = i + j*LocalConfig::width;
      fs << covariance[index];
    }
  }
  fs << "]";
  fs.release();
  ROS_INFO_STREAM("Wrote statistics to " << LocalConfig::error_filename);
}

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);
  ros::init(argc, argv, "stereo_error_characterizer",ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  tf::TransformBroadcaster tfbc;
  br = &tfbc; 
  tf::TransformListener tfls;
  listener = &tfls; 
  ros::Rate rate(1.0f / LocalConfig::detection_interval);

  std::string left_image_topic;
  if (LocalConfig::left_image_topic != "") {
    left_image_topic = LocalConfig::left_image_topic;
  }
  else {
    if (LocalConfig::rect) {
      left_image_topic = std::string(LocalConfig::stereo_name).append("image_mono");
    } else {
      left_image_topic = std::string(LocalConfig::stereo_name).append("image_rect");
    }
  }

  std::string right_image_topic;
  if (LocalConfig::right_image_topic != "") {
    right_image_topic = LocalConfig::right_image_topic;
  }
  else {
    if (LocalConfig::rect) {
      right_image_topic = std::string(LocalConfig::stereo_name).append("image_mono");
    } else {
      right_image_topic = std::string(LocalConfig::stereo_name).append("image_rect");
    }
  }

  left_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(LocalConfig::left_pose_topic,1);
  right_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(LocalConfig::right_pose_topic,1);

  std::string left_info_topic;
  if (LocalConfig::left_info_topic != "") {
    left_info_topic = LocalConfig::left_info_topic;
  } else {
    left_info_topic = "camera_info";
  }
  std::cout << left_info_topic << std::endl;

  std::string right_info_topic;
  if (LocalConfig::right_info_topic != "") {
    right_info_topic = LocalConfig::right_info_topic;
  } else {
    right_info_topic = "camera_info";
  }
  std::cout << right_info_topic << std::endl;

  // wait for camera info
  sensor_msgs::CameraInfoConstPtr left_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(left_info_topic, nh, ros::Duration(1));
  if (!left_info_ptr) throw std::runtime_error("could not get left camera info");

  sensor_msgs::CameraInfoConstPtr right_info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(right_info_topic, nh, ros::Duration(1));
  if (!right_info_ptr) throw std::runtime_error("could not get right camera info");

  stereo_model.fromCameraInfo(left_info_ptr, right_info_ptr);

  if (LocalConfig::rect) {
    ROS_INFO_STREAM("Rectifying image data from topic " << left_image_topic);
  } else {
    ROS_INFO_STREAM("Not rectifying image data from topic " << left_image_topic);
  }

  // set up the camera matrices
  cv::Mat_<double> leftProjMatrix(3,4,  const_cast<double*>(&left_info_ptr->P[0]));
  cv::Mat_<double> leftCameraMatrix(3, 3, const_cast<double*>(&left_info_ptr->K[0]));
  cv::Mat_<double> leftDistCoeffs;
  if (LocalConfig::rect) {
    leftDistCoeffs = cv::Mat_<double>(5, 1, const_cast<double*>(&left_info_ptr->D[0]));
  }
  else {
    leftCameraMatrix = leftProjMatrix(cv::Rect(0,0,3,3)).clone(); // use the optimal K matrix for the virtual rectified camera when the incoming image is already rectified
    leftDistCoeffs = cv::Mat_<double>(cv::Size(5,1), 0);
  }

  cv::Mat_<double> rightProjMatrix(3,4,  const_cast<double*>(&right_info_ptr->P[0]));
  cv::Mat_<double> rightCameraMatrix(3, 3, const_cast<double*>(&right_info_ptr->K[0]));
  cv::Mat_<double> rightDistCoeffs;
  if (LocalConfig::rect) {
    rightDistCoeffs = cv::Mat_<double>(5, 1, const_cast<double*>(&right_info_ptr->D[0]));
  } else {
    rightCameraMatrix = rightProjMatrix(cv::Rect(0,0,3,3)).clone(); // use the optimal K matrix for the virtual rectified camera when the incoming image is already rectified
    rightDistCoeffs = cv::Mat_<double>(cv::Size(5,1), 0);
  }

  left_camera_matrix = leftCameraMatrix.clone();
  right_camera_matrix = rightCameraMatrix.clone();
  left_dist_coeff = leftDistCoeffs.clone();
  right_dist_coeff = rightDistCoeffs.clone();
  std::cout << left_camera_matrix << std::endl;
  std::cout << right_camera_matrix << std::endl;

  // set up the windows
  left_window_name = nh.resolveName(LocalConfig::left_pose_topic,true);
  left_window_name.erase(left_window_name.begin()); //remove leading slash
  std::string sep("::");
  while (true) {
    size_t ind = left_window_name.find('/');
    if (ind == std::string::npos) { break; }
    left_window_name.replace(ind,1,sep);
  }
  cv::namedWindow(left_window_name.c_str(), 1 );

  right_window_name = nh.resolveName(LocalConfig::right_pose_topic,true);
  right_window_name.erase(right_window_name.begin()); //remove leading slash
  while (true) {
    size_t ind = right_window_name.find('/');
    if (ind == std::string::npos) { break; }
    right_window_name.replace(ind,1,sep);
  }
  cv::namedWindow(right_window_name.c_str(), 1 );

  // set up the statistics buffers
  num_samples = 0;
  actual_corners = calcChessboardCorners(cv::Size(LocalConfig::width, LocalConfig::height), LocalConfig::square);
  for (unsigned int i = 0; i < actual_corners.size(); i++) {
    avg_error.push_back(cv::Point3f(0,0,0));
    rms_error.push_back(cv::Point3f(0,0,0));
    covariance.push_back(cv::Mat::zeros(3,3,CV_32F));
  }

  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, left_image_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub(nh, right_image_topic, 1);

  // PositionErrorTable pet("stereo_measurement_errors", "stereo_errors");
  // PositionMeasurementNoise n = pet.LookupNoise(cv::Point3f(-0.044, -0.025, 0.555));
  // std::cout << n.avgError << std::endl;
  // std::cout << n.covariance << std::endl;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mySyncPolicy;
  message_filters::Synchronizer<mySyncPolicy> sync(mySyncPolicy(10), left_image_sub, right_image_sub);
  sync.registerCallback(boost::bind(&image_callback, _1, _2));

  while (ros::ok()) {
    ros::spinOnce();
    int key = cv::waitKey(10);
    if (key == 'q') {
      SaveErrorXML();
      ros::shutdown();
    }

    rate.sleep();
  }

  SaveErrorXML();
  return 0;
}

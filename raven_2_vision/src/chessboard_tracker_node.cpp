#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <LinearMath/btTransform.h>
#include "raven_2_vision/raven_vision.h"
#include <ros/ros.h>
#include "config.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/topic.h>

using namespace std;
using namespace cv;
#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

struct LocalConfig : Config {
	static int width;
	static int height;
	static float square;
	static string topic;
	//static string imageNS;
	LocalConfig() : Config() {
		params.push_back(new Parameter<int>("width", &width, "chessboard width"));
		params.push_back(new Parameter<int>("height", &height, "chessboard height"));
		params.push_back(new Parameter<float>("square", &square, "chessboard sidelength"));
		params.push_back(new Parameter<string>("topic", &topic, "pose topic"));
		//params.push_back(new Parameter<string>("imageNS", &imageNS, "image namespace"));
	}
};
int LocalConfig::width = 6;
int LocalConfig::height = 8;
float LocalConfig::square = .01;
string LocalConfig::topic = "chessboard_pose";
//string LocalConfig::imageNS = "/usb_cam";

static sensor_msgs::ImageConstPtr last_msg;
bool message_pending = false;
void callback(const sensor_msgs::ImageConstPtr& msg) {
	last_msg = msg;
	message_pending = true;
}


int main(int argc, char* argv[]) {
	Parser parser;
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);
	ros::init(argc, argv, "chessboard_tracker_node");
	ros::NodeHandle nh;


	//ros::Subscriber image_sub = nh.subscribe(LocalConfig::imageNS + "/image_rect", 1, callback);
	ros::Subscriber image_sub = nh.subscribe("image_rect", 1, callback);
	//ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/chessboard",1);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(LocalConfig::topic,1);

	//sensor_msgs::CameraInfoConstPtr info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(LocalConfig::imageNS + "/camera_info", nh, ros::Duration(1));
	sensor_msgs::CameraInfoConstPtr info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", nh, ros::Duration(1));
	if (!info_ptr) throw runtime_error("could not get camera info");
	cv::Mat_<double> cameraMatrix(3,3, const_cast<double*>(&info_ptr->K[0]));
//	cv::Mat_<double> distCoeffs(5,1, &info_ptr->D[0]);
	cv::Mat_<double> distCoeffs(cv::Size(5,1), 0);


	const char* windowName = "Image View";
	namedWindow(windowName, 1 );

	geometry_msgs::PoseStamped ps;
	while (ros::ok()) {

		while (true) {
			ros::spinOnce();
			if (message_pending) {
				message_pending = false;
				break;
			}
			else if (!ros::ok()) return 1;
			else sleep(.01);
		}

		cv::Mat image = cv_bridge::toCvCopy(last_msg)->image;


		bool gotPose = getChessboardPose(image, Size(LocalConfig::width, LocalConfig::height), LocalConfig::square, cameraMatrix, distCoeffs, ps.pose, true);
		if (gotPose) {
			ROS_INFO("found chessboard");
			ps.header.frame_id = last_msg->header.frame_id;
			ps.header.stamp = ros::Time::now();
			pose_pub.publish(ps);
		}


		imshow(windowName, image);
		int key = waitKey(10);
		if (key == 'q') break;


	}



}

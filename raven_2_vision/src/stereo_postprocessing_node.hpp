/*
 * Node to decimate and compute disparity maps from the Prosilica images. Created because I'm fed up with trying to get the
 * stupid ROS image pipeline to do what I want to do. We can't compute disparity maps on the raw Prosilica images because they're too big,
 * and all we really need to do is make them smaller before stereo processing but this is clearly too much to ask in ROS. 
 *
 * @author Jeff Mahler
 * @contact jmahler@berkeley.edu
 */

#include "config.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <stereo_image_proc/DisparityConfig.h>

#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <raven_2_vision/PostprocessingConfig.h>

#include <ros/ros.h>
#include <ros/topic.h>

class StereoPostprocessingNode
{
  typedef dynamic_reconfigure::Server<raven_2_vision::PostprocessingConfig> ReconfigureServer;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> mySyncPolicy;
  typedef message_filters::Synchronizer<mySyncPolicy> ApproximateSync;

 public:
  StereoPostprocessingNode(ros::NodeHandle* nh);
  ~StereoPostprocessingNode() {}

 public:
  void init();
  void image_callback(const sensor_msgs::ImageConstPtr& left_image_msg,
                      const sensor_msgs::CameraInfoConstPtr& left_info_msg,
                      const sensor_msgs::ImageConstPtr& right_image_msg,
                      const sensor_msgs::CameraInfoConstPtr& right_info_msg);
  void config_callback(raven_2_vision::PostprocessingConfig &config, uint32_t level);

 private:
  // subscribers
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;

  // publishers
  ros::Publisher disparity_pub_;
  ros::Publisher depth_pub_;

  // dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  // processing variables
  cv::StereoBM block_matcher_; // contains scratch buffers for block matching
  image_geometry::StereoCameraModel stereo_model_;

  // node handle
  ros::NodeHandle* nh_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // other params
  int decimation_x_;
  int decimation_y_;
  int kernel_size_;
  float kernel_sigma_;
};

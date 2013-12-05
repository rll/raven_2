#include "stereo_postprocessing_node.hpp"

#include <iostream>
#include <fstream>
#include <time.h> 

#include <LinearMath/btTransform.h>
#include <sensor_msgs/image_encodings.h>

#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

struct LocalConfig : Config {
  static std::string left_image_topic;
  static std::string right_image_topic;
  static std::string left_info_topic;
  static std::string right_info_topic;
  static std::string disparity_topic;
  static std::string depth_topic;
  static int queue_size;

  LocalConfig() : Config() {
    params.push_back(new Parameter<std::string>("left_image", &left_image_topic, "left image topic"));
    params.push_back(new Parameter<std::string>("right_image", &right_image_topic, "right image topic"));
    params.push_back(new Parameter<std::string>("left_camera_info", &left_info_topic, "left camera info topic"));
    params.push_back(new Parameter<std::string>("right_camera_info", &right_info_topic, "right camera info topic"));
    params.push_back(new Parameter<std::string>("disparity", &disparity_topic, "disparity topic"));
    params.push_back(new Parameter<std::string>("depth", &depth_topic, "depth topic"));
    params.push_back(new Parameter<int>("queue_size", &queue_size, "queue size"));
  }
};

std::string LocalConfig::left_image_topic = "left/image_rect";
std::string LocalConfig::right_image_topic = "right/image_rect";
std::string LocalConfig::left_info_topic = "left/camera_info";
std::string LocalConfig::right_info_topic = "right/camera_info";
std::string LocalConfig::disparity_topic = "downsampled_disparity";
std::string LocalConfig::depth_topic = "downsampled_depth";
int LocalConfig::queue_size = 5;


StereoPostprocessingNode::StereoPostprocessingNode(ros::NodeHandle* nh)
  : nh_(nh)
{
  init();
}

void StereoPostprocessingNode::init()
{
  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f = boost::bind(&StereoPostprocessingNode::config_callback, this, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer());
  reconfigure_server_->setCallback(f);

  // set up the subscribers
  std::string left_image_topic = LocalConfig::left_image_topic;
  std::string right_image_topic = LocalConfig::right_image_topic;
  std::string left_info_topic = LocalConfig::left_info_topic;
  std::string right_info_topic = LocalConfig::right_info_topic;
  left_image_sub_.subscribe(*nh_, left_image_topic, 1);
  right_image_sub_.subscribe(*nh_, right_image_topic, 1);
  left_info_sub_.subscribe(*nh_, left_info_topic, 1);
  right_info_sub_.subscribe(*nh_, right_info_topic, 1);

  // set up the publishers
  std::string disparity_topic = LocalConfig::disparity_topic;
  disparity_pub_ = nh_->advertise<stereo_msgs::DisparityImage>(disparity_topic, 1);

  std::string depth_topic = LocalConfig::depth_topic;
  depth_pub_ = nh_->advertise<sensor_msgs::Image>(depth_topic, 1);

  approximate_sync_.reset(new ApproximateSync(mySyncPolicy(LocalConfig::queue_size),
                                              left_image_sub_, left_info_sub_,
                                              right_image_sub_, right_info_sub_));
  approximate_sync_->registerCallback(boost::bind(&StereoPostprocessingNode::image_callback, this, _1, _2, _3, _4));


  // set default
}

void StereoPostprocessingNode::config_callback(raven_2_vision::PostprocessingConfig &config, uint32_t level)
{
  decimation_x_ = config.decimation_x;
  decimation_y_ = config.decimation_y;
  kernel_size_ = config.kernel_size | 0x1; // must be odd
  kernel_sigma_ = config.kernel_sigma;

  // Tweak all settings to be valid
  config.prefilter_size |= 0x1; // must be odd
  config.correlation_window_size |= 0x1; // must be odd
  config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

  // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
  // concurrently, so this is thread-safe.
  block_matcher_.state->preFilterSize       = config.prefilter_size;
  block_matcher_.state->preFilterCap        = config.prefilter_cap;
  block_matcher_.state->SADWindowSize       = config.correlation_window_size;
  block_matcher_.state->minDisparity        = config.min_disparity;
  block_matcher_.state->numberOfDisparities = config.disparity_range;
  block_matcher_.state->uniquenessRatio     = config.uniqueness_ratio;
  block_matcher_.state->textureThreshold    = config.texture_threshold;
  block_matcher_.state->speckleWindowSize   = config.speckle_size;
  block_matcher_.state->speckleRange        = config.speckle_range;
}

void StereoPostprocessingNode::image_callback(const sensor_msgs::ImageConstPtr& left_image_msg,
                                              const sensor_msgs::CameraInfoConstPtr& left_info_msg,
                                              const sensor_msgs::ImageConstPtr& right_image_msg,
                                              const sensor_msgs::CameraInfoConstPtr& right_info_msg)
{
  // Make sure we have the kind of images we want
  assert(left_image_msg->encoding == sensor_msgs::image_encodings::MONO8);
  assert(right_image_msg->encoding == sensor_msgs::image_encodings::MONO8);

  // Update the camera model
  stereo_model_.fromCameraInfo(left_info_msg, right_info_msg);

  // Read in params
  unsigned int dec_width = left_image_msg->width / decimation_x_;
  unsigned int dec_height = left_image_msg->height / decimation_y_;

  // Allocate new disparity image message
  stereo_msgs::DisparityImagePtr disp_msg = boost::make_shared<stereo_msgs::DisparityImage>();
  disp_msg->header         = left_info_msg->header;
  disp_msg->image.header   = left_info_msg->header;
  disp_msg->image.height   = dec_height;
  disp_msg->image.width    = dec_width;
  disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg->image.step     = disp_msg->image.width * sizeof(float);
  disp_msg->image.data.resize(disp_msg->image.height * disp_msg->image.step);

  // Stereo parameters
  disp_msg->f = stereo_model_.right().fx() / decimation_x_;
  disp_msg->T = stereo_model_.baseline();

  // Allocate new depth image message
  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
  depth_msg->header        = left_info_msg->header;
  depth_msg->height        = dec_height;
  depth_msg->width         = dec_width;
  depth_msg->encoding      = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg->step           = depth_msg->width * sizeof(float);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);

  // Compute window of (potentially) valid disparities
  cv::Ptr<CvStereoBMState> params = block_matcher_.state;
  int border   = params->SADWindowSize / 2;
  int left   = params->numberOfDisparities + params->minDisparity + border - 1;
  int wtf = (params->minDisparity >= 0) ? border + params->minDisparity : std::max(border, -params->minDisparity);
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border;
  disp_msg->valid_window.x_offset = left;
  disp_msg->valid_window.y_offset = top;
  disp_msg->valid_window.width    = right - left;
  disp_msg->valid_window.height   = bottom - top;

  // Disparity search range
  disp_msg->min_disparity = params->minDisparity;
  disp_msg->max_disparity = params->minDisparity + params->numberOfDisparities - 1;
  disp_msg->delta_d = 1.0 / 16; // OpenCV uses 16 disparities per pixel

  // Create cv::Mat views onto all buffers
  cv::Mat_<uint8_t> left_image(left_image_msg->height, left_image_msg->width,
                               const_cast<uint8_t*>(&left_image_msg->data[0]),
                               left_image_msg->step);
  cv::Mat_<uint8_t> right_image(right_image_msg->height, right_image_msg->width,
                                const_cast<uint8_t*>(&right_image_msg->data[0]),
                                right_image_msg->step);
  cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                             reinterpret_cast<float*>(&disp_msg->image.data[0]),
                             disp_msg->image.step);

  // Downsample
  cv::Mat_<uint8_t> left_image_ds;
  cv::Mat_<uint8_t> right_image_ds;
  cv::Size ds_size(dec_width, dec_height);

  if (dec_width < left_image_msg->width && dec_height < left_image_msg->height) {

    cv::GaussianBlur(left_image, left_image, cv::Size(kernel_size_, kernel_size_),
                     kernel_sigma_, kernel_sigma_); 
    cv::GaussianBlur(right_image, right_image, cv::Size(kernel_size_, kernel_size_),
                     kernel_sigma_, kernel_sigma_); 
    cv::resize(left_image, left_image_ds, ds_size, 0, 0, cv::INTER_NEAREST); 
    cv::resize(right_image, right_image_ds, ds_size, 0, 0, cv::INTER_NEAREST); 
  }
  else {
    // do not allow upscaling of images, enforce same size
    left_image_ds = left_image.clone();
    right_image_ds = right_image.clone();
  }

  // Perform block matching to find the disparities
  block_matcher_(left_image_ds, right_image_ds, disp_image, CV_32F);

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  
  double cx_l = stereo_model_.left().cx() / decimation_x_;
  double cx_r = stereo_model_.right().cx() / decimation_x_;
  if (cx_l != cx_r)
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);

  cv::Mat disp_image_resized;
  cv::Mat point_cloud_resized;
  cv::Mat XYZ[3];
  cv::Mat_<float> depth_image(depth_msg->height, depth_msg->width,
                              reinterpret_cast<float*>(&depth_msg->data[0]),
                              depth_msg->step);
  XYZ[2] = depth_image;

  cv::resize(disp_image, disp_image_resized, cv::Size(left_image_msg->width, left_image_msg->height)); 
  stereo_model_.projectDisparityImageTo3d(disp_image_resized, point_cloud_resized);
  cv::resize(point_cloud_resized, point_cloud_resized, cv::Size(depth_msg->width, depth_msg->height));
  cv::split(point_cloud_resized, XYZ);
  
  disparity_pub_.publish(disp_msg);
  depth_pub_.publish(depth_msg);
}

int main(int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  ros::init(argc, argv, "stereo_postprocessing");
  ros::NodeHandle nh;
  StereoPostprocessingNode stereoNode(&nh);

  ros::spin();
  return 0;
}


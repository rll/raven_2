#include "ros/ros.h"
#include "sixense/Calib.h"
#include "visualization_msgs/Marker.h"
namespace vm = visualization_msgs;

struct HydraViz {
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
  HydraViz(ros::NodeHandle& nh) : 
    m_nh(nh),
    m_pub(nh.advertise<visualization_msgs::Marker>("hydra_marker", 1000)),
    m_sub(nh.subscribe("hydra_calib", 100, &HydraViz::callback, this))
  {
  }

  void callback(const sixense::Calib& msg) {
    for (int i=0; i < 2; i++) {
      const sixense::CalibPaddle& paddle = msg.paddles[i];
      vm::Marker marker;
      marker.action = vm::Marker::ADD;
      marker.id = i;
      marker.header.frame_id = "/torso_lift_link";
      marker.header.stamp = ros::Time::now()- ros::Duration(.1);
      marker.type = vm::Marker::CUBE;
      marker.pose.position.x = paddle.transform.translation.x;
      marker.pose.position.y = paddle.transform.translation.y;
      marker.pose.position.z = paddle.transform.translation.z;
      marker.pose.orientation = paddle.transform.rotation;
      marker.scale.x = .3;
      marker.scale.y = .3;
      marker.scale.z = .3;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1;
      m_pub.publish(marker);
    }
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "hydra_viz");
  ros::NodeHandle n;


  HydraViz viz(n);
  ros::spin();
  return 0;
}

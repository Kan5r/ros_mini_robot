#pragma once

#include "encorder.hpp"
#include <ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace ros 
{
class Odometry
{
  friend void enc0Callback();
  friend void enc1Callback();
  friend void enc2Callback();
  friend void enc3Callback();

public:
  static Odometry& getInstance() {static Odometry odom; return odom;}
  void init();
  void run();
  ros::Encorder enc_[4];
private:
  Odometry();
  ~Odometry();
  void calculateOdom();
  void broadcastTransform();
  ros::NodeHandle nh_;
  tf::TransformBroadcaster odom_broadcaster_;
  ros::Publisher pose2d_pub_;
  geometry_msgs::Pose2D pose2d_;
  geometry_msgs::TransformStamped odom_trans;
};
}



#include "odometry.hpp"
#include <new>

using ros::Odometry;

//////////Parameters///////////////////////////////////
namespace {
constexpr int ENC_A_[4] = {2, 4, 6, 8};
constexpr int ENC_B_[4] = {3, 5, 7, 9};
constexpr int PPR_ = 400;
constexpr double TIRE_RADIUS_ = 0.024; //[m]
constexpr double DIAGONAL2_ = 0.2; //[m] 
constexpr double ROOT2_ = 1.41421356;
}
/////////////////////////////////////////////////////

void enc0Callback()
{
  Odometry::getInstance().enc_[0].countPulseCallback();
}

void enc1Callback()
{
  Odometry::getInstance().enc_[1].countPulseCallback();
}

void enc2Callback()
{
  Odometry::getInstance().enc_[2].countPulseCallback();
}

void enc3Callback()
{
  Odometry::getInstance().enc_[3].countPulseCallback();
}

Odometry::Odometry() :
  pose2d_pub_("pose", &pose2d_)
{
  init();
  pose2d_.x = pose2d_.y = pose2d_.theta = 0.0;
}

Odometry::~Odometry() {}

void Odometry::init()
{
  void (*encCallback[])() = {enc0Callback, enc1Callback, enc2Callback, enc3Callback};
  for (int i = 0; i < 4; i++) {
    new(enc_ + i) ros::Encorder(ENC_A_[i], ENC_B_[i], ros::Multiplication::M4);
    enc_[i].init();
    attachInterrupt(digitalPinToInterrupt(ENC_A_[i]), encCallback[i], CHANGE); 
    attachInterrupt(digitalPinToInterrupt(ENC_B_[i]), encCallback[i], CHANGE);
  }
  
  nh_.initNode();
  nh_.advertise(pose2d_pub_);
  odom_broadcaster_.init(nh_);
}

void Odometry::run()
{
  calculateOdom();
  broadcastTransform();
  pose2d_pub_.publish(&pose2d_);
  nh_.spinOnce(); 
}

void Odometry::calculateOdom()
{
  static int pulse[4] = {}, pulse_prev[4] = {};
  double v[4], vx, vy, vw;

  noInterrupts();
  for (int i = 0; i < 4; i++) {
    pulse[i] = enc_[i].getPulse(); 
    v[i] = (pulse[0] - pulse_prev[0]) * 2 * PI * TIRE_RADIUS_ / PPR_;
    pulse_prev[i] = pulse[i];
  }
  interrupts();

  vx = (-v[0]/ROOT2_ + v[1]/ROOT2_ + v[2]/ROOT2_ - v[3]/ROOT2_) / 4;
  vy = (-v[0]/ROOT2_ - v[1]/ROOT2_ + v[2]/ROOT2_ + v[3]/ROOT2_) / 4;
  vw = (v[0] + v[1] + v[2] + v[3]) / 4 / DIAGONAL2_;
  
  pose2d_.x += vx*cos(pose2d_.theta) - vy*sin(pose2d_.theta);
  pose2d_.y += vx*sin(pose2d_.theta) + vy*cos(pose2d_.theta);
  pose2d_.theta += vw;
}

void Odometry::broadcastTransform()
{
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = pose2d_.x;
  odom_trans.transform.translation.y = pose2d_.y;

  odom_trans.transform.rotation = tf::createQuaternionFromYaw(pose2d_.theta);
  odom_trans.header.stamp = nh_.now();

  odom_broadcaster_.sendTransform(odom_trans);
}
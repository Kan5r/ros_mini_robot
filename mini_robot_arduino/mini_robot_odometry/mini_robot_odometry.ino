#include "odometry.hpp"

void setup() {
  Serial.begin(115200);
  ros::Odometry::getInstance().init();
}

void loop() {
  ros::Odometry::getInstance().run();
}

#include "gps_mockup_sensor.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_mockup_sensor");

  ros::NodeHandle nodeHandle;

  gps_mockup_sensor::Sensor sensor;

  sensor.spin();

  return EXIT_SUCCESS;
}

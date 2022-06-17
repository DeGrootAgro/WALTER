#include "mockup_sensors/collection_mockup_sensor.hpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "collection_mockup_sensor");

  collection_mockup_sensor::Sensor sensor;

  sensor.spin();

  return EXIT_SUCCESS;
}

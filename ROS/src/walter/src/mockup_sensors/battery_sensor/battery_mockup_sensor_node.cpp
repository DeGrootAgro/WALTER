#include "mockup_sensors/battery_mockup_sensor.hpp"

#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "battery_mockup_sensor");

  battery_mockup_sensor::Sensor sensor;

  sensor.spin();

  return EXIT_SUCCESS;
}

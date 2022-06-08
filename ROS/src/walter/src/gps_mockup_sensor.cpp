#include "gps_mockup_sensor.hpp"
#include <ros/ros.h>

namespace gps_mockup_sensor {

Sensor::Sensor(double frequency = 1.0) { loop_rate = new ros::Rate(frequency); }
Sensor::~Sensor() { ; }

void Sensor::spin() {
  while (ros::ok()) {
    loop_rate->sleep();
  };
}

} // namespace gps_mockup_sensor

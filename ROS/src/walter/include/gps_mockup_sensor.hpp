#ifndef __gps_mockup_sensor
#define __gps_mockup_sensor

#include <ros/ros.h>

namespace gps_mockup_sensor {
class Sensor {
private:
  ros::Rate *loop_rate;

public:
  Sensor(double frequency = 1.0);
  ~Sensor();

  void spin();
};
} // namespace gps_mockup_sensor

#endif
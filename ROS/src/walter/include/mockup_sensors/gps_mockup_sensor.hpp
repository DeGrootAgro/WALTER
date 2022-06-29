#ifndef __gps_mockup_sensor
#define __gps_mockup_sensor

#include <ros/ros.h>

namespace gps_mockup_sensor {
class Sensor {
private:
  ros::Rate *loop_rate;
  ros::NodeHandle *node_handle;
  ros::Publisher publisher;
  int sequence;
  double longitude, latitude, altitude, noise_factor;

public:
  Sensor();
  ~Sensor();

  double calculate_noise();

  void spin();
};
} // namespace gps_mockup_sensor

#endif
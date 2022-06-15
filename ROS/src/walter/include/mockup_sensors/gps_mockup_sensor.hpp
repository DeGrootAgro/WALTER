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
  double latitude, longitude, altitude;

public:
  Sensor();
  ~Sensor();

  void spin();
};
} // namespace gps_mockup_sensor

#endif
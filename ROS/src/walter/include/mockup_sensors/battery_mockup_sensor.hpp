#ifndef __battery_mockup_sensor
#define __battery_mockup_sensor

#include <ros/ros.h>

namespace battery_mockup_sensor {
class Sensor {
private:
  ros::Rate *loop_rate;
  ros::NodeHandle *node_handle;
  ros::Publisher publisher;
  int sequence;

  double battery_usage_per_second, percentage;
  ros::Time last_time;

public:
  Sensor();
  ~Sensor();

  void spin();
};
} // namespace battery_mockup_sensor

#endif
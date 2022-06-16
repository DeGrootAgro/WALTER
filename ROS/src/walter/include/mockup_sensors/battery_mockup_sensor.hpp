#ifndef __battery_mockup_sensor
#define __battery_mockup_sensor

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <walter/ChargeBatteryAction.h>

namespace battery_mockup_sensor {
class Sensor {
private:
  ros::Rate *loop_rate;
  ros::NodeHandle *node_handle;
  ros::Publisher publisher;
  actionlib::SimpleActionServer<walter::ChargeBatteryAction> *action_server;
  int sequence;
  bool is_charging;

  double battery_usage_per_second, percentage,
      battery_charging_speed_per_second, charging_target;
  ros::Time last_sensor_cycle;

public:
  Sensor();
  ~Sensor();

  void execute(const walter::ChargeBatteryActionGoal &goal);

  void spin();
};
} // namespace battery_mockup_sensor

#endif
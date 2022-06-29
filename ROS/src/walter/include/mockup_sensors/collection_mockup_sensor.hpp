#ifndef __collection_mockup_sensor
#define __collection_mockup_sensor

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <walter_msgs/EmptyCollectionAction.h>

namespace collection_mockup_sensor {
class Sensor {
private:
  ros::Rate *loop_rate;
  ros::NodeHandle *node_handle;
  ros::Publisher publisher;
  actionlib::SimpleActionServer<walter_msgs::EmptyCollectionAction>
      *action_server;
  int sequence;
  bool is_emptying;

  double nut_collection_speed_per_second, percentage,
      collection_emptying_speed_per_second, charging_target;
  ros::Time last_sensor_cycle;

public:
  Sensor();
  ~Sensor();

  void execute(const walter_msgs::EmptyCollectionActionGoal &goal);

  void spin();
};
} // namespace collection_mockup_sensor

#endif
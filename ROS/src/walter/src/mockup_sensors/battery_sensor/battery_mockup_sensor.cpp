#include "mockup_sensors/battery_mockup_sensor.hpp"
#include <ros/ros.h>
#include <walter/Battery.h>
#include <walter/ChargeBatteryAction.h>

namespace battery_mockup_sensor {

Sensor::Sensor() {
  this->node_handle = new ros::NodeHandle("~");

  this->node_handle->param("/walter/mockup_sensors/battery_usage",
                           this->battery_usage_per_second, 0.1);

  this->node_handle->param("/walter/mockup_sensors/battery_charging_speed",
                           this->battery_charging_speed_per_second, 2.0);

  int frequency;
  this->node_handle->param("/walter/mockup_sensors/battery_frequency",
                           frequency, 1);

  this->loop_rate = new ros::Rate(frequency);

  std::string topic("/walter/battery_percentage");
  this->node_handle->param("/walter/mockup_sensors/battery_topic", topic,
                           topic);

  std::string action_server_name("battery_action_server");
  this->node_handle->param("/walter/mockup_sensors/battery_action_server_topic",
                           topic, topic);

  this->publisher = node_handle->advertise<walter::Battery>(topic, 1000);
  this->sequence = 1;

  this->percentage = 100.0;

  this->action_server =
      new actionlib::SimpleActionServer<walter::ChargeBatteryAction>(
          action_server_name, false);
  this->action_server->start();

  this->last_sensor_cycle = ros::Time::now();
}

Sensor::~Sensor() { ; }

void Sensor::spin() {
  // Create header for message
  std_msgs::Header header;
  header.frame_id = "battery_percentage";

  // Create battery msg
  walter::Battery battery_msg;

  while (ros::ok()) {
    // Set sequence in header and increment sequence
    header.seq = sequence++;
    // Set the time this message is being constructed
    header.stamp = ros::Time::now();
    // Set header of nav_sat_fix
    battery_msg.header = header;

    // Calculate the time between last and now
    ros::Time current_time = ros::Time::now();
    ros::Duration difference = current_time - this->last_sensor_cycle;

    // Update the last time
    last_sensor_cycle = ros::Time::now();

    // Calculate the amount of percentage that has been used
    double seconds = difference.toSec();
    this->percentage -= seconds * this->battery_usage_per_second;

    // Check if new charging goal is available
    if (this->action_server->isNewGoalAvailable()) {
      this->is_charging = true;
      this->charging_target =
          (*this->action_server->acceptNewGoal()).target_percentage;
    }

    // Check if the battery is being charged
    if (this->is_charging) {
      // Add the amount of charge
      this->percentage += seconds * this->battery_charging_speed_per_second;

      // Check if the stop condition is achieved
      if (charging_target < this->percentage) {
        this->is_charging = false;
        this->action_server->setSucceeded();
      }
    }

    // Set the percentage
    battery_msg.percentage = this->percentage;

    // Publish the msg
    publisher.publish(battery_msg);

    // Sleep for the remaining time
    loop_rate->sleep();
  };
}
} // namespace battery_mockup_sensor

#include "mockup_sensors/collection_mockup_sensor.hpp"
#include <ros/ros.h>
#include <walter/Collection.h>
#include <walter/EmptyCollectionAction.h>

namespace collection_mockup_sensor {

Sensor::Sensor() {
  this->node_handle = new ros::NodeHandle("~");

  this->node_handle->param("/collection_speed",
                           this->nut_collection_speed_per_second, 0.1);

  this->node_handle->param("/collection_emptying_speed",
                           this->collection_emptying_speed_per_second, 2.0);

  int frequency;
  this->node_handle->param("/frequency", frequency, 1);

  this->loop_rate = new ros::Rate(frequency);

  std::string topic("/walter/collection_percentage");
  this->node_handle->param("/collection_sensor_topic", topic, topic);

  std::string action_server_name("collection_action_server");
  this->node_handle->param("/action_server_topic", topic, topic);

  this->publisher = node_handle->advertise<walter::Collection>(topic, 1000);
  this->sequence = 1;

  this->percentage = 100.0;
  this->charging_target = 0.0;
  this->is_emptying = false;

  this->action_server =
      new actionlib::SimpleActionServer<walter::EmptyCollectionAction>(
          action_server_name, false);
  this->action_server->start();

  this->last_sensor_cycle = ros::Time::now();
}

Sensor::~Sensor() { ; }

void Sensor::spin() {
  // Create header for message
  std_msgs::Header header;
  header.frame_id = "collection_percentage";

  // Create battery msg
  walter::Collection collection_msg;

  while (ros::ok()) {
    // Set sequence in header and increment sequence
    header.seq = sequence++;
    // Set the time this message is being constructed
    header.stamp = ros::Time::now();
    // Set header of nav_sat_fix
    collection_msg.header = header;

    // Calculate the time between last and now
    ros::Time current_time = ros::Time::now();
    ros::Duration difference = current_time - this->last_sensor_cycle;

    // Update the last time
    last_sensor_cycle = ros::Time::now();

    // Calculate the amount of percentage that has been used
    double seconds = difference.toSec();
    this->percentage -= seconds * this->nut_collection_speed_per_second;

    // Check if new charging goal is available
    if (this->action_server->isNewGoalAvailable()) {
      this->is_emptying = true;
      this->charging_target =
          (*this->action_server->acceptNewGoal()).target_percentage;
    }

    // Check if the battery is being charged
    if (this->is_emptying) {
      // Add the amount of charge
      this->percentage += seconds * this->collection_emptying_speed_per_second;

      // Check if the stop condition is achieved
      if (charging_target < this->percentage) {
        this->is_emptying = false;
        this->action_server->setSucceeded();
      }
    }

    // Set the percentage
    collection_msg.percentage = this->percentage;

    // Publish the msg
    publisher.publish(collection_msg);

    // Sleep for the remaining time
    loop_rate->sleep();
  };
}
} // namespace collection_mockup_sensor

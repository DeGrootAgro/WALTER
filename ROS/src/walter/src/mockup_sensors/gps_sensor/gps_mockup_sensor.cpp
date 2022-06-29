#include "mockup_sensors/gps_mockup_sensor.hpp"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

namespace gps_mockup_sensor {

Sensor::Sensor() {
  this->node_handle = new ros::NodeHandle("~");

  this->node_handle->param("longitude", this->longitude, 0.0);
  this->node_handle->param("latitude", this->latitude, 0.0);
  this->node_handle->param("altitude", this->altitude, 0.0);
  this->node_handle->param("noise_factor", this->noise_factor, 0.0001);

  int frequency;
  this->node_handle->param("frequency", frequency, 1);

  this->loop_rate = new ros::Rate(frequency);

  std::string topic("/gps/fix");
  this->node_handle->param("topic", topic, topic);

  this->publisher = node_handle->advertise<sensor_msgs::NavSatFix>(topic, 1000);
  this->sequence = 1;
}
Sensor::~Sensor() { ; }

void Sensor::spin() {
  // Create header for message
  std_msgs::Header header;
  header.frame_id = "gps";

  // Create nav sat status message
  sensor_msgs::NavSatStatus nav_sat_status;
  nav_sat_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  nav_sat_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  // Create nav sat fix message
  sensor_msgs::NavSatFix nav_sat_fix;
  nav_sat_fix.position_covariance_type =
      sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  nav_sat_fix.status = nav_sat_status;

  while (ros::ok()) {
    // Set sequence in header and increment sequence
    header.seq = sequence++;

    // Set the coordinates with a noise
    nav_sat_fix.latitude = this->latitude + calculate_noise();
    nav_sat_fix.longitude = this->longitude + calculate_noise();
    nav_sat_fix.altitude = this->altitude + calculate_noise();

    // Set the time this message is being constructed
    header.stamp = ros::Time::now();
    // Set header of nav_sat_fix
    nav_sat_fix.header = header;

    publisher.publish(nav_sat_fix);

    // Sleep for the remaining time
    loop_rate->sleep();
  };
}

double Sensor::calculate_noise() {
  // Generate a random number between -100 and 100
  int random = rand() % 201 - 100;
  // Make noise equal 1%
  double noise = this->noise_factor / 100;

  // Multiply 1% by random witch is mapped between -100 and 100.
  noise *= random;

  return noise;
}

} // namespace gps_mockup_sensor

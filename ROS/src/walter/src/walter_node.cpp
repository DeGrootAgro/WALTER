#include <ros/console.h>
#include <ros/ros.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "Walter");
  ros::NodeHandle nh;
  ros::Rate loop_rate(0.5);

  while (ros::ok()) {

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

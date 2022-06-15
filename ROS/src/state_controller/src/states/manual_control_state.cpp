#include "state_controller/robot_state.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

void manual_control_state::on_cmd_vel_recieve(const geometry_msgs::Twist::ConstPtr& msg){
     if (is_active)
     {
       cmd_vel_publisher->publish(msg);
     }
    
}

manual_control_state::manual_control_state(ros::NodeHandle *n, ros::Publisher *cmd_vel_publisher) : robot_state(n, cmd_vel_publisher){
   ros::Subscriber sub = n->subscribe("manual_control/cmd_vel",1000,&manual_control_state::on_cmd_vel_recieve, this);
    
}


void manual_control_state::update(){


}


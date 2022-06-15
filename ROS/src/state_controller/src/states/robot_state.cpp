#include "state_controller/robot_state.h"
#include "ros/ros.h"

robot_state::robot_state(ros::NodeHandle *n,ros::Publisher *cmd_vel_publisher){
    is_active = false;
    node = n;
    this->cmd_vel_publisher = cmd_vel_publisher;
}

void robot_state::enter_state(){
    is_active = true;
}

void robot_state::exit_state(){
    is_active = false;
}
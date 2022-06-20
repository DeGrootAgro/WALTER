#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


/*
THIS CLASS IS UNUSED
*/


class robot_state{
    public:
     bool is_active;
     ros::NodeHandle *node;
     ros::Publisher *cmd_vel_publisher;

     void update();
     void enter_state();
     void exit_state();
     void on_cmd_vel_recieve(const geometry_msgs::Twist::ConstPtr& msg);
     robot_state(ros::NodeHandle *n, ros::Publisher *cmd_vel_publisher);

};

class manual_control_state : public robot_state{
    public:
        void update();
        manual_control_state(ros::NodeHandle *n, ros::Publisher *cmd_vel_publisher);
        void on_cmd_vel_recieve(const geometry_msgs::Twist::ConstPtr& msg);

};

class automatic_control_state : public robot_state{

};

class navigate_to_loading_state : public robot_state{

};

class empty_load_state : public robot_state{

};

class navigate_to_charging_state : public robot_state{

};

class charging_state : public robot_state{

};

class idle_state : public robot_state{

};

class emergency_stop_state : public robot_state{
    public:
        void update();
        emergency_stop_state(ros::NodeHandle *n, ros::Publisher *cmd_vel_publisher);
        void on_cmd_vel_recieve(const geometry_msgs::Twist::ConstPtr& msg);
};


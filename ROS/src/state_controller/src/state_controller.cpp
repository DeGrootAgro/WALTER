#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"


#include <sstream>

enum state{
    ST_IDLE,
    ST_CHARGING,
    ST_NAVIGATE_TO_CHARGING,
    ST_MANUAL_CONTROL,
    ST_COLLECTING,
    ST_NAVIGATE_TO_LOADING,
    ST_LOADING,
    ST_EMERGENCY_STOP
};

state current_state;

void on_cmd_vel_recieve_manual(const geometry_msgs::Twist::ConstPtr& msg){
    if (current_state == ST_MANUAL_CONTROL)
    {
        /* code */
    }
    
}

void on_cmd_vel_recieve_automatic(const geometry_msgs::Twist::ConstPtr& msg){
   if (current_state == ST_COLLECTING)
   {
       /* code */
   }
   


}

void on_cmd_vel_recieve_navigate_to(const geometry_msgs::Twist::ConstPtr& msg){
   
}





int main(int argc, char **argv){
    ros::init(argc, argv, "state_controller");

    ros::NodeHandle n;

    ros::Publisher cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    current_state = ST_IDLE;







} 
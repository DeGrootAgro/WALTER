#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"


#include <sstream>

void log(std::string data){
    std::cout<<"[STATE_CONTROLLER] " << data << std::endl;
}



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

ros::Publisher cmd_vel_publisher;
ros::Publisher paused_publisher;

std::map<state, std::string> enum_names;

void init_state_names(){
    enum_names[ST_IDLE]                 = "ST_IDLE";
    enum_names[ST_CHARGING]             = "ST_CHARGING";
    enum_names[ST_NAVIGATE_TO_CHARGING] = "ST_NAVIGATE_TO_CHARGING";
    enum_names[ST_MANUAL_CONTROL]       = "ST_MANUAL_CONTROL";
    enum_names[ST_COLLECTING]           = "ST_COLLECTING";
    enum_names[ST_NAVIGATE_TO_LOADING]  = "ST_NAVIGATE_TO_LOADING";
    enum_names[ST_LOADING]              = "ST_LOADING";
    enum_names[ST_EMERGENCY_STOP]       = "ST_EMERGENCY_STOP"; 
}



void set_nav_goal(){

}

void run_coverage_config(){
    std_msgs::Bool paused;
 
    if (current_state == ST_COLLECTING)
    {
       paused.data = false;

    }else
    {
        paused.data = true;
    }
    paused_publisher.publish(paused);
    
}

void on_cmd_vel_recieve_manual(const geometry_msgs::Twist::ConstPtr& msg){
    if (current_state == ST_MANUAL_CONTROL)
    {
        cmd_vel_publisher.publish(msg);
    }
}

void on_cmd_vel_recieve_automatic(const geometry_msgs::Twist::ConstPtr& msg){
   if (current_state == ST_COLLECTING)
   {
        cmd_vel_publisher.publish(msg);
   }
}

void on_cmd_vel_recieve_navigate_to(const geometry_msgs::Twist::ConstPtr& msg){
   if (current_state == ST_NAVIGATE_TO_CHARGING || current_state == ST_NAVIGATE_TO_LOADING)
   {
      cmd_vel_publisher.publish(msg);
   }
}

void on_recieve_state_switch(const std_msgs::String::ConstPtr &msg){
    log("Revieved state switch message");
    std::string newStateString(msg->data.c_str());
    for(auto& it : enum_names){
        if (it.second == newStateString)
        {/* condition */
            std::cout << "[STATE_CONTROLLER] switching from " << enum_names[current_state] << " to " << newStateString << std::endl;

            current_state = it.first;

            break;
        }
    }
}


int main(int argc, char **argv){
  
    log("initializing");

    ros::init(argc, argv, "state_controller");
    ros::NodeHandle n;
    init_state_names();

    ros::Subscriber manual_cmd_vel_subscriber       = n.subscribe("/manual/cmd_vel",1000,on_cmd_vel_recieve_manual);
    ros::Subscriber automatic_cmd_vel_subscriber    = n.subscribe("/automatic/cmd_vel",1000, on_cmd_vel_recieve_automatic);
    ros::Subscriber navigation_cmd_vel_subscriber   = n.subscribe("/navigate/cmd_vel",1000,on_cmd_vel_recieve_navigate_to);
    ros::Subscriber state_subscriber                = n.subscribe("/state_controller/state",1000,on_recieve_state_switch);


    geometry_msgs::Twist twist_idle;

    twist_idle.angular.x = 0;
    twist_idle.angular.y = 0;
    twist_idle.angular.z = 0;

    twist_idle.linear.x = 0;
    twist_idle.linear.y = 0;
    twist_idle.linear.z = 0;
    

    cmd_vel_publisher   = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    paused_publisher    = n.advertise<std_msgs::Bool>("/pause",1000);


    cmd_vel_publisher.publish(twist_idle);
    current_state = ST_IDLE;


    std::cout << "[STATE_CONTROLLER] Current state: " << enum_names[current_state] << std::endl; 

    
    ros::Rate rate(15);
    while (ros::ok())
    {
        run_coverage_config();


       switch (current_state)
       {
        case ST_IDLE:
            cmd_vel_publisher.publish(twist_idle);
            break;

        case ST_CHARGING:
            cmd_vel_publisher.publish(twist_idle);
            break;

        case ST_EMERGENCY_STOP:
            cmd_vel_publisher.publish(twist_idle);
            break;

        case ST_LOADING:
            break;

        case ST_MANUAL_CONTROL:
            break;

        case ST_COLLECTING:
            break;

        case ST_NAVIGATE_TO_CHARGING:
            break;

        case ST_NAVIGATE_TO_LOADING:
            break;
       default:
           break;
       }
       ros::spinOnce();
       rate.sleep();
    }
} 
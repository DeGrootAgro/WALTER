#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <boost/thread/thread.hpp>

#define MAX_LINEAR_SPEED    0.5
#define MAX_ANGULAR_SPEED   0.5

#include <sstream>


void log(std::string data){
    std::cout<<"[STATE_CONTROLLER] " << data << std::endl;
}


//this boolean reports if walter has changed state since te last iteration of the loop
bool switched = false;

//all walter states
enum state{
    ST_IDLE,
    ST_CHARGING,
    ST_NAVIGATE_TO_CHARGING,
    ST_MANUAL_CONTROL,
    ST_COLLECTING,
    ST_NAVIGATE_TO_LOADING,
    ST_LOADING,
    ST_EMERGENCY_STOP,
    ST_RETURN_TO_PATH
};

state current_state;

//all publishers
ros::Publisher cmd_vel_publisher;
ros::Publisher paused_publisher;
ros::Publisher goal_publisher;
ros::Publisher change_state_publisher;

//defines the location of the robot the instant it switches from ST_COLLECTING to another state
//this is necessary when trying to continue collecting after charging or unloading.
geometry_msgs::Pose lastRoutePose; 

//define all states as strings for logging purposes
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
    enum_names[ST_RETURN_TO_PATH]       = "ST_RETURN_TO_PATH";
}

//sets the nav goal for the robot on the map
void set_nav_goal(ros::NodeHandle n, geometry_msgs::Pose p){
    geometry_msgs::PoseStamped pose;

    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose = p;

    
    goal_publisher.publish(pose);
}

//sends cmd vel data to the robot. also checks if maximum speed is exceeded
void send_CMD_VEL_data(geometry_msgs::Twist::ConstPtr t){
   geometry_msgs::Twist newTwist;
   
   newTwist.angular = t->angular;
   newTwist.linear = t->linear; 

    if (newTwist.linear.x > MAX_LINEAR_SPEED)
    {
            newTwist.linear.x = MAX_LINEAR_SPEED;
    }
    else if (newTwist.linear.x < -MAX_LINEAR_SPEED)
    {
        newTwist.linear.x = -MAX_LINEAR_SPEED;
    }

    if (newTwist.angular.z > MAX_ANGULAR_SPEED)
    {
        newTwist.angular.z = MAX_ANGULAR_SPEED;
    }else if (newTwist.angular.z < -MAX_ANGULAR_SPEED)
    {
        newTwist.angular.z < -MAX_ANGULAR_SPEED;
    }
    
    cmd_vel_publisher.publish(newTwist);
}

//this function pauses the path planner if the robot is not in the ST_COLLECTING STATE
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


//these functions are the different functions for revieving the cmd vel for the different states

//ST_MANUAL_CONTROL
void on_cmd_vel_recieve_manual(const geometry_msgs::Twist::ConstPtr& msg){
    if (current_state == ST_MANUAL_CONTROL)
    {
        send_CMD_VEL_data(msg);
    }
}

//ST_COLLECTING
void on_cmd_vel_recieve_automatic(const geometry_msgs::Twist::ConstPtr& msg){
   if (current_state == ST_COLLECTING)
   {
       send_CMD_VEL_data(msg);
   }
}

//ST_NAVIGATE_TO_CHARGING, ST_NAVIGATE_TO_LOADING or ST_RETURN_TO_PATH
void on_cmd_vel_recieve_navigate_to(const geometry_msgs::Twist::ConstPtr& msg){
   if (current_state == ST_NAVIGATE_TO_CHARGING || current_state == ST_NAVIGATE_TO_LOADING|| current_state == ST_RETURN_TO_PATH)
   {
     send_CMD_VEL_data(msg);
   }
}


//if the state needs to be swicthed, this function is called
void on_recieve_state_switch(const std_msgs::String::ConstPtr &msg){
    log("Revieved state switch message");
    std::string newStateString(msg->data.c_str());
    for(auto& it : enum_names){
        if (it.second == newStateString)
        {/* condition */
            if (current_state == ST_COLLECTING)
            {
                 lastRoutePose = *ros::topic::waitForMessage<geometry_msgs::Pose>("robot_pose");
            }

            std::cout << "[STATE_CONTROLLER] switching from " << enum_names[current_state] << " to " << newStateString << std::endl;
            current_state = it.first;
            switched = true;
          
            
            


            break;
        }
    }
}

//this function checks if the nav goal is reached
void on_recieve_nav_status(const move_base_msgs::MoveBaseActionResult &msg){
   int status = msg.status.status;

   if(status == msg.status.SUCCEEDED){
       std::string newState;
     
       switch (current_state)
       {
       case ST_RETURN_TO_PATH:
            newState = enum_names[ST_COLLECTING];          
           break;
        case ST_NAVIGATE_TO_CHARGING:
             newState = enum_names[ST_CHARGING];  
        break;
        case ST_NAVIGATE_TO_LOADING:
            newState = enum_names[ST_LOADING];
        break;
        
       default: 
        return;
       
       }
        std_msgs::String msg;
        std::stringstream ss;
        ss << newState;
        msg.data = ss.str();
        change_state_publisher.publish(msg);
   }

}


//publishes the current thread once per secons
void publish_state_thread(){

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Publisher curent_state_publsiher = node->advertise<std_msgs::String>("/current_state",1000);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << enum_names[current_state];
    msg.data = ss.str();

    curent_state_publsiher.publish(msg);
    
    loop_rate.sleep();
  }

}


int main(int argc, char **argv){
  
    log("initializing");

    ros::init(argc, argv, "state_controller");
    ros::NodeHandle n;


    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("/enable_control");
    init_state_names();


    //setup subscribers
    ros::Subscriber manual_cmd_vel_subscriber       = n.subscribe("/manual/cmd_vel",1000,on_cmd_vel_recieve_manual);
    ros::Subscriber automatic_cmd_vel_subscriber    = n.subscribe("/automatic/cmd_vel",1000, on_cmd_vel_recieve_automatic);
    ros::Subscriber navigation_cmd_vel_subscriber   = n.subscribe("/navigate/cmd_vel",1000,on_cmd_vel_recieve_navigate_to);
    ros::Subscriber state_subscriber                = n.subscribe("/state_controller/state",1000,on_recieve_state_switch);
    ros::Subscriber nav_status_subscriber           = n.subscribe("/move_base/result",1000, on_recieve_nav_status);

    //set up publishers
    cmd_vel_publisher       = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    paused_publisher        = n.advertise<std_msgs::Bool>("/pause",1000);
    goal_publisher          = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
    change_state_publisher  = n.advertise<std_msgs::String>("/state_controller/state",1000);
  
    //define stopped twist message
    geometry_msgs::Twist twist_idle;

    twist_idle.angular.x = 0;
    twist_idle.angular.y = 0;
    twist_idle.angular.z = 0;

    twist_idle.linear.x = 0;
    twist_idle.linear.y = 0;
    twist_idle.linear.z = 0;
    
    cmd_vel_publisher.publish(twist_idle);
    current_state = ST_IDLE;


    std::cout << "[STATE_CONTROLLER] Current state: " << enum_names[current_state] << std::endl; 
    boost::thread thread_b(publish_state_thread);


    //garantee that the loop runs at 15hz
    //most robots require a frequency of at least 10hz
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
        case ST_RETURN_TO_PATH:
            if (switched)
            {
                //set goal to last saved point
                set_nav_goal(n,lastRoutePose);
                switched = false;

            }
            

        break;

        case ST_NAVIGATE_TO_CHARGING:
            if (switched)
            {

                //set goal to the charging point   
                geometry_msgs::Pose p = *(ros::topic::waitForMessage<geometry_msgs::Pose>("UNLOAD_POINT"));

                set_nav_goal(n,p);
                switched = false;
        }
            break;

        case ST_NAVIGATE_TO_LOADING:
            if (switched)
            {
                //set goal to the unloading point
                geometry_msgs::Pose p = *(ros::topic::waitForMessage<geometry_msgs::Pose>("UNLOAD_POINT"));

                set_nav_goal(n,p);
                switched = false;
                
            }
            
            break;
       default:
           break;
       }
               

       ros::spinOnce();
       rate.sleep();
    }
    thread_b.join();
} 
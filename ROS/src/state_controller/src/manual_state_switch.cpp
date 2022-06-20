#include "ros/ros.h"
#include "std_msgs/String.h"


//all posible walter states
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
void log(std::string data){
    std::cout<<"[MANUAL_STATE_SWITCH] "<< data <<std::endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv,"manual_state_switch");
    ros::NodeHandle n;

    init_state_names();


        for(auto& it : enum_names){
            log(it.second);
        }

    ros::Publisher state_switcher = n.advertise<std_msgs::String>("/state_controller/state",1000);

    while(ros::ok()){
        std::cout<<"[MANUAL_STATE_SWITCH] new state?"<<std::endl;
        std::string answer;
        std::cin >> answer;

        for(auto& it : enum_names){
            if (it.second == answer)
                {
               
                std_msgs::String msg;

                std::stringstream ss;
                ss << answer;
                msg.data = ss.str();

                state_switcher.publish(msg);

                std::cout<<"[MANUAL_STATE_SWITCH] publishing " << answer <<   std::endl;

                goto endloop;
                }
        }
         std::cout<<"[MANUAL_STATE_SWITCH] state not found" <<   std::endl;
        endloop:
            ;
       
    }

}
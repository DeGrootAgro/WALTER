#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <vector>
#include "landmark.h"



void log(std::string data){
    std::cout << "[WALTER_LANDMARK_MANAGER] " << data <<std::endl;
}


void save_position(landmark lm ){

}

std::vector<landmark> createFakeLocations(){
    geometry_msgs::PoseStamped ps;
    ps.pose.orientation.w = 1;
    ps.pose.orientation.x = -5.5;
    
    ps.pose.position.x = -2;
    ps.pose.position.y = -0.5;

    landmark unload_point(ps, "UNLOAD_POINT");

    std::vector<landmark> lm;
    lm.push_back(unload_point);

    return lm;


}


std::vector<landmark> load_positions(){
    return createFakeLocations();
}



int main(int argc, char** argv){
    ros::init(argc, argv, "walter_landmark_manager");
    ros::NodeHandle n;

    if (argc == 0)
    {
        log("\nuse -save {LOCATION_NAME} to save a location\n use -publish to start publishing landmarks");
        ros::shutdown();
    }
    if (argv[0] == "-save")
    {
        //sla op in config
        if (argc >=2)
        {
            /* code */
            std::string name = argv[1];
            nav_msgs::Odometry location = *ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");

            geometry_msgs::PoseStamped ps;

            ps.pose = location.pose.pose;
            landmark lm(ps,name);

            save_position(lm);

        }
        

    }
    if (argv[0] == "-publish")
    {
        ros::Rate r = ros::Rate(1);

        std::vector<landmark> landmarks = load_positions();
        std::vector<ros::Publisher> publishers;

        for (int i = 0; i < landmarks.size(); i++)
        {
            n.advertise()   
        }
        

        while (ros::ok())
        {


            ros::spinOnce();
            r.sleep();
        }
        
    }
    
    
    
}
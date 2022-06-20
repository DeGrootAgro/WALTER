#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

/*
the landmark class defines a point that can be used by the system. this can be a charging point or an unloading point for example.

*/
class landmark
{
private:
   geometry_msgs::Pose _location;
   std::string _name;
   ros::Publisher p;
   
public:
    landmark(geometry_msgs::Pose location, std::string name, ros::Publisher pub);
    std::string name();
    geometry_msgs::Pose location();
    void publish();
};

landmark::landmark(geometry_msgs::Pose location, std::string name, ros::Publisher pub)
{
    this->_location = location;
    this->_name = name;
    this->p = pub;
}

geometry_msgs::Pose landmark::location(){
    return _location;
}

std::string landmark::name(){
    return _name;
}

/*
Publish the location to ROS
*/
void landmark::publish(){
    p.publish(_location);
}



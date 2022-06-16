#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"


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
void landmark::publish(){
    p.publish(_location);
}



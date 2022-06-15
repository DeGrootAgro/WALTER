#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


class landmark
{
private:
   geometry_msgs::PoseStamped _location;
   std::string _name;
public:
    landmark(geometry_msgs::PoseStamped location, std::string name);
    std::string name();
    geometry_msgs::PoseStamped location();
};

landmark::landmark(geometry_msgs::PoseStamped location, std::string name)
{
    this->_location = location;
    this->_name = name;
}

geometry_msgs::PoseStamped landmark::location(){
    return _location;
}

std::string landmark::name(){
    return _name;
}



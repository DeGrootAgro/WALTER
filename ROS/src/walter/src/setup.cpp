#include "boustrophedon_msgs/PlanMowingPathAction.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <actionlib/client/simple_action_client.h>
#include <stdexcept>

int main(int argc, char **argv) {
  ros::init(argc, argv, "Walter");

  ros::spin();
}

// Robot waits for baseplate command
// Robot will save coordinates of baseplate
// Wait for message that boundry construction has started
// Save all coordinates for constructing the boundry
// Wait for message end of boundry or coordinates overlap with begin point
// (Predefined readius) Wait for specify no go zones or continue command

// In case of no go zones repeat first three actions with different command

// END OF MANUAL PART
// Robot will drive inside boundy for scanning exploring map

// Save map using map server
// construct path using Boustrophedon planner
// Overlay coordinates with map so map is based on coordinates
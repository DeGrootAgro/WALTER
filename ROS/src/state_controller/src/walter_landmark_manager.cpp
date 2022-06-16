#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "state_controller/landmark.h"
#include <vector>
#include "std_msgs/String.h"

void log(std::string data) {
  std::cout << "[WALTER_LANDMARK_MANAGER] " << data << std::endl;
}

void save_position(landmark lm) {}

std::vector<landmark> createFakeLocations(ros::NodeHandle n) {
  geometry_msgs::Pose ps;

  ps.orientation.w = 1;
  ps.orientation.x = 0;
  ps.orientation.y = 0;
  ps.orientation.z = 0;

  ps.position.x = 0.68;
  ps.position.y = -1.82;
  ps.position.z = 0;

 

  landmark unload_point(ps, "UNLOAD_POINT", n.advertise<geometry_msgs::Pose>("UNLOAD_POINT",1000));

  std::vector<landmark> lm;
  lm.push_back(unload_point);

  return lm;
}

std::vector<landmark> load_positions(ros::NodeHandle n) { return createFakeLocations(n); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "walter_landmark_manager");
  ros::NodeHandle n;

  log("checking args");
  std::cout << argc << std::endl;
  log(argv[0]);
 

  if (argc == 1) {
   log("publishing");
    ros::Rate r = ros::Rate(1);

    std::vector<landmark> landmarks = load_positions(n);
    //std::vector<ros::Publisher> publishers;

   

    while (ros::ok()) {

      for (int i = 0; i < landmarks.size(); i++) {
        landmarks[i].publish();
      }

      ros::spinOnce();
      r.sleep();
    }
  }

}
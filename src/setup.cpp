#include "boustrophedon_msgs/PlanMowingPathAction.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include <actionlib/client/simple_action_client.h>
#include <stdexcept>

typedef actionlib::SimpleActionClient<boustrophedon_msgs::PlanMowingPathAction>
    Client;

// The code snippet below is licensed under CC0 1.0.
template <typename... Args>
std::string string_format(const std::string &format, Args... args) {
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) +
               1; // Extra space for '\0'
  if (size_s <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(),
                     buf.get() + size - 1); // We don't want the '\0' inside
}

geometry_msgs::Point32 createPoint32(double x = 0, double y = 0, double z = 0) {
  geometry_msgs::Point32 p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Point createPoint(double x = 0, double y = 0, double z = 0) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Quaternion createOrientation(double x = 0, double y = 0,
                                            double z = 0, double w = 1) {
  geometry_msgs::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

visualization_msgs::Marker
createMarker(std::string ns,
             decltype(visualization_msgs::Marker::ARROW) marker_type =
                 visualization_msgs::Marker::ARROW,
             double x = 0, double y = 0, double z = 0) {
  // Set our initial shape type to be a cube

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID Any marker sent with the same namespace and id will overwrite the old
  // one
  marker.ns = ns;
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and
  // SPHERE, ARROW, and CYLINDER
  marker.type = marker_type;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  return marker;
}

void pathPlannerCallBack(
    const boustrophedon_msgs::PlanMowingPathActionResult &msg) {
  std::vector<boustrophedon_msgs::StripingPoint> points =
      msg.result.plan.points;

  ros::NodeHandle nodeHandle;

  ros::Publisher vis_pub = nodeHandle.advertise<visualization_msgs::Marker>(
      "visualization_marker", 0);

  visualization_msgs::Marker outlineMarker =
      createMarker("outline", visualization_msgs::Marker::LINE_STRIP);
  std_msgs::ColorRGBA boundryColor;

  // Construct the boundy color
  boundryColor.r = 1;
  boundryColor.g = boundryColor.b = 0;
  boundryColor.a = 0.5;

  outlineMarker.color = boundryColor;
  std::vector<visualization_msgs::Marker> markers;

  visualization_msgs::Marker currentMarker;
  geometry_msgs::Point lastEndPoint = createPoint();
  int lineSegment = 1;

  for (int pointItterator = 0; pointItterator < points.size();
       pointItterator++) {
    boustrophedon_msgs::StripingPoint point = points.at(pointItterator);
    switch (point.type) {
    case boustrophedon_msgs::StripingPoint::OUTLINE: {
      outlineMarker.points.push_back(
          createPoint(point.point.x, point.point.y, point.point.z));
      break;
    }

    case boustrophedon_msgs::StripingPoint::STRIPE_START: {
      std::string ns = string_format("Segment %d", lineSegment++);

      visualization_msgs::Marker marker =
          createMarker(ns, visualization_msgs::Marker::ARROW);
      currentMarker = marker;

      geometry_msgs::Point position =
          createPoint(point.point.x, point.point.y, point.point.z);

      if (lastEndPoint != createPoint()) {
        std::string connection_ns = string_format("Segment %d", lineSegment++);
        // Marker used to navigate from one end to the start of the new segment
        visualization_msgs::Marker connectionMarker =
            createMarker(connection_ns, visualization_msgs::Marker::ARROW);
        connectionMarker.points.push_back(lastEndPoint);
        connectionMarker.points.push_back(position);
        markers.push_back(connectionMarker);
      }

      currentMarker.points.push_back(position);
      break;
    }

    case boustrophedon_msgs::StripingPoint::STRIPE_INTERMEDIATE: {
      currentMarker.points.push_back(
          createPoint(point.point.x, point.point.y, point.point.z));
      break;
    }
    case boustrophedon_msgs::StripingPoint::STRIPE_END: {
      geometry_msgs::Point position =
          createPoint(point.point.x, point.point.y, point.point.z);
      currentMarker.points.push_back(position);
      markers.push_back(currentMarker);
      lastEndPoint = position;
      break;
    }

    default:
      ROS_ERROR("Unknown error type given from path planner!");
      break;
    }
  }

  ros::Rate rate(1.0);
  while (ros::ok()) {
    vis_pub.publish(outlineMarker);

    for (int i = 0; i < markers.size(); i++) {
      vis_pub.publish(markers.at(i));
    }

    rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Walter");
  ros::NodeHandle n;

  ros::Subscriber sub =
      n.subscribe("/plan_path/result", 1000, pathPlannerCallBack);

  ros::Rate rate(1.);

  std::vector<geometry_msgs::Point32> points;
  points.push_back(createPoint32(-3, 3));
  points.push_back(createPoint32(3, 3));
  points.push_back(createPoint32(3, -3));
  points.push_back(createPoint32(-3, -3));

  geometry_msgs::Polygon polygon;
  polygon.points = points;

  std_msgs::Header header;
  header.frame_id = "walter";
  header.seq = 1;
  header.stamp = ros::Time();

  geometry_msgs::PolygonStamped polygonStamped;
  polygonStamped.polygon = polygon;
  polygonStamped.header = header;

  // Pose creation
  geometry_msgs::Pose pose;
  pose.orientation = geometry_msgs::Quaternion();
  pose.position = createPoint(0, 0, 0);
  pose.orientation = createOrientation();

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header = header;
  poseStamped.pose = pose;

  Client client("plan_path", true);
  client.waitForServer(ros::Duration(5.0));
  boustrophedon_msgs::PlanMowingPathGoal goal;
  goal.robot_position = poseStamped;
  goal.property = polygonStamped;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  printf("Current State: %s\n", client.getState().toString().c_str());
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
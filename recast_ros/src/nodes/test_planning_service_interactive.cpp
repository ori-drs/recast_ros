#include "recast_ros/RecastPathSrv.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class TestPlanningServiceInteractive
{
public:
  TestPlanningServiceInteractive(ros::NodeHandle& node_handle) : node_handle_(node_handle)
  {
    // ros params
    node_handle_.param("target_topic", target_topic_, std::string("/move_base_simple/goal"));
    node_handle_.param("path_service", path_service_, std::string("/recast_node/plan_path"));
    node_handle_.param("loop_rate", loop_rate_, 1.0);

    // subscribe target
    subscriber_ = node_handle_.subscribe(target_topic_, 1, &TestPlanningServiceInteractive::callback, this);
  }
  void callback(const geometry_msgs::PoseStamped& msg)
  {
    ROS_INFO("Received point: %f %f %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    target_ = msg.pose.position;
  }
  void run()
  {
    // ros
    ros::ServiceClient client_recast = node_handle_.serviceClient<recast_ros::RecastPathSrv>(path_service_);

    // loop
    ros::Rate loop_rate(loop_rate_);
    while (ros::ok()) {

      // get current state. TODO: take as param?
      geometry_msgs::Point current;
      current.x = 0;
      current.y = 0;
      current.z = 0;

      // recast plan to goal
      recast_ros::RecastPathSrv srv;
      srv.request.startXYZ = current;
      srv.request.goalXYZ = target_;
      if (!client_recast.call(srv)) {
        ROS_ERROR("Failed to call recast query service");
        continue;
      }
      if (srv.response.path.size() < 2) {
        ROS_ERROR("Could not find a path");
        continue;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
protected:
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_;
  std::string target_topic_;
  std::string path_service_;
  double loop_rate_;
  geometry_msgs::Point target_;
};

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "high_level_planner_node");
  ros::NodeHandle node_handle("~");
  TestPlanningServiceInteractive test_planning_service_interactive(node_handle);
  test_planning_service_interactive.run();
  return 0;
}


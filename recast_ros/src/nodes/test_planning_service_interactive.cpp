//
// Copyright (c) 2019 Martim Brandão martim@robots.ox.ac.uk, Omer Burak Aladag aladagomer@sabanciuniv.edu, Ioannis Havoutis ioannis@robots.ox.ac.uk
// As a part of Dynamic Robot Systems Group, Oxford Robotics Institute, University of Oxford
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//   claim that you wrote the original software. If you use this software
//   in a product, an acknowledgment in the product documentation would be
//   appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//   misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.

#include "recast_ros/RecastPathSrv.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

class TestPlanningServiceInteractive
{
public:
  TestPlanningServiceInteractive(ros::NodeHandle &node_handle) : node_handle_(node_handle)
  {
    // ros params
    node_handle_.param("target_topic", target_topic_, std::string("/move_base_simple/goal"));
    node_handle_.param("path_service", path_service_, std::string("/recast_node/plan_path"));
    node_handle_.param("loop_rate", loop_rate_, 1.0);

    // subscribe target
    subscriber_ = node_handle_.subscribe(target_topic_, 1, &TestPlanningServiceInteractive::callback, this);
  }
  void callback(const geometry_msgs::PoseStamped &msg)
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
    while (ros::ok())
    {

      // get current state. TODO: take as param?
      geometry_msgs::Point current;
      current.x = 0;
      current.y = 0;
      current.z = 0;

      // recast plan to goal
      recast_ros::RecastPathSrv srv;
      srv.request.startXYZ = current;
      srv.request.goalXYZ = target_;
      if (!client_recast.call(srv))
      {
        ROS_ERROR("Failed to call recast query service");
        // continue;
      }
      if (srv.response.path.size() < 2)
      {
        ROS_ERROR("Could not find a path");
        //  continue;
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "high_level_planner_node");
  ros::NodeHandle node_handle("~");
  TestPlanningServiceInteractive test_planning_service_interactive(node_handle);
  test_planning_service_interactive.run();
  return 0;
}

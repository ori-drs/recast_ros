#include "recast_ros/RecastPlanner.h"
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
#include <fstream>


ros::Publisher marker_pub;


struct RecastTest
{
  RecastTest(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
  {
    // ros params
    std::string base = ros::package::getPath("recast_demos");
    nodeHandle_.param("path", path_, base+std::string("/data/map.obj"));
    nodeHandle_.param("path_areas", pathAreas_, base+std::string("/data/map.dat"));
    nodeHandle_.param("start_x", startX_, 0.0);
    nodeHandle_.param("start_y", startY_, 0.0);
    nodeHandle_.param("start_z", startZ_, 0.0);
    nodeHandle_.param("goal_x", goalX_, 0.0);
    nodeHandle_.param("goal_y", goalY_, 0.0);
    nodeHandle_.param("goal_z", goalZ_, 0.0);
    // recast settings
    recast_.stg.cellSize = 0.1f;
    recast_.stg.cellHeight = 0.1f;
    recast_.stg.agentHeight = 0.7f;
    recast_.stg.agentRadius = 0.2f;
    recast_.stg.agentMaxClimb = 0.41f;
    recast_.stg.agentMaxSlope = 60.0f;
  }
  void setVisualParameters(visualization_msgs::Marker &v)
  {
    v.header.frame_id = "/my_frame";
    v.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    v.ns = "Triangle";
    v.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    v.action = visualization_msgs::Marker::ADD;
    v.pose.position.x = 0;
    v.pose.position.y = 0;
    v.pose.position.z = 0;
    v.pose.orientation.x = 0.0;
    v.pose.orientation.y = 0.0;
    v.pose.orientation.z = 0.0;
    v.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    v.scale.x = 1.0;
    v.scale.y = 1.0;
    v.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    v.color.r = 0.0f;
    v.color.g = 1.0f;
    v.color.b = 0.0f;
    v.color.a = 1.0;

    v.lifetime = ros::Duration();
  }
  void test()
  {
    // load
    pcl::PolygonMesh pclMesh;
    std::vector<char> trilabels;
    visualization_msgs::Marker triList;
    triList.type = visualization_msgs::Marker::TRIANGLE_LIST;
    setVisualParameters(triList);

    pcl::io::loadPolygonFileOBJ(path_, pclMesh); // pcl::io::loadPolygonFilePLY(path_, pclMesh);
    ROS_INFO("loaded OBJ file");
    loadAreas(pathAreas_, trilabels);
    ROS_INFO("loaded AREAS file");
    // build NavMesh. TODO: use trilabels in the construction
    if (!recast_.build(pclMesh, trilabels)) {
      ROS_ERROR("Could not build NavMesh");
      return;
    }
    // test path planning (Detour)
    pcl::PointXYZ start;
    start.x = startX_;
    start.y = startY_;
    start.z = startZ_;
    pcl::PointXYZ goal;
    goal.x = goalX_;
    goal.y = goalY_;
    goal.z = goalZ_;
    std::vector<pcl::PointXYZ> path;
    pcl::PointCloud<pcl::PointXYZ> tri_verts;

    pcl::fromPCLPointCloud2(pclMesh.cloud, tri_verts);
    geometry_msgs::Point p;

    size_t i;

    for(i = 0; i < tri_verts.size() ; i++)
    {
      p.x = tri_verts.at(i).x;
      p.y = -tri_verts.at(i).z; 
      p.z = tri_verts.at(i).y;

      triList.points.push_back(p);
    }

    std::cout << i << "\n";

  while(ros::ok())
  {
     while (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }



    marker_pub.publish(triList);
  }

    if (!recast_.query(start, goal, path)) { // TODO: this function should not use pcl as arguments but, std::vector or Eigen...
      ROS_ERROR("Could not obtain shortest path");
      return;
    }
    ROS_INFO("success");
    for (unsigned int i = 0; i < path.size(); i++) {
      ROS_INFO("path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
    }




  }
 /*bool loadAreas(const std::string& path, std::vector<char>& labels) // moved to RecastsPlanner.h / .cpp
  {
    // load file with per-triangle area types
    std::ifstream INFILE(path, std::ios::in | std::ifstream::binary);
    std::istreambuf_iterator<char> eos;
    std::istreambuf_iterator<char> iter(INFILE);
    std::copy(iter, eos, std::back_inserter(labels));
    return true;   
  }
  */
  ros::NodeHandle nodeHandle_;
  std::string path_;
  std::string pathAreas_;
  recastapp::RecastPlanner recast_;
  double startX_;
  double startY_;
  double startZ_;
  double goalX_;
  double goalY_;
  double goalZ_;
};


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "recast_test_node");
  ros::NodeHandle nodeHandle("~");
  marker_pub = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  RecastTest test(nodeHandle);
  test.test();
  return 0;
}


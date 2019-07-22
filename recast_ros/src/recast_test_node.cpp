#include "recast_ros/RecastPlanner.h"
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
#include <fstream>

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
  void test()
  {
    // load
    pcl::PolygonMesh pclMesh;
    std::vector<char> trilabels;
    pcl::io::loadPolygonFileOBJ(path_, pclMesh); // pcl::io::loadPolygonFilePLY(path_, pclMesh);
    ROS_INFO("loaded OBJ file");
    loadAreas(pathAreas_, trilabels);
    ROS_INFO("loaded AREAS file");
    // build NavMesh. TODO: use trilabels in the construction
    if (!recast_.build(pclMesh)) {
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
    if (!recast_.query(start, goal, path)) { // TODO: this function should not use pcl as arguments but, std::vector or Eigen...
      ROS_ERROR("Could not obtain shortest path");
      return;
    }
    ROS_INFO("success");
    for (unsigned int i = 0; i < path.size(); i++) {
      ROS_INFO("path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
    }
  }
  bool loadAreas(const std::string& path, std::vector<char>& labels)
  {
    // load file with per-triangle area types
    std::ifstream INFILE(path, std::ios::in | std::ifstream::binary);
    std::istreambuf_iterator<char> eos;
    std::istreambuf_iterator<char> iter(INFILE);
    std::copy(iter, eos, std::back_inserter(labels));    
  }
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
  RecastTest test(nodeHandle);
  test.test();
  return 0;
}


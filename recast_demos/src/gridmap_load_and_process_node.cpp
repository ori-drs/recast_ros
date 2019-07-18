#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <ros/package.h>

int main (int argc, char* argv[])
{
  ros::init(argc, argv, "gridmap_save_node");
  ros::NodeHandle nodeHandle("~");

  std::string base = ros::package::getPath("recast_demos");
  std::string path;
  std::string topic;
  nodeHandle.param("pathToBag", path, base+std::string("/data/map.gridmap"));
  nodeHandle.param("topic", topic, std::string("gridmap"));

  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::loadFromBag(path, topic, gridMap);

  // TODO: process map

  return 0;
}


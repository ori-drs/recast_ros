// Copyright (c) 2019 Martim Brandão martim@robots.ox.ac.uk, Omer Burak Aladag aladagomer@sabanciuniv.edu, Ioannis Havoutis havoutis@robots.ox.ac.uk
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

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <ros/package.h>

bool convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, grid_map::GridMap &gridmap, double resolution)
{
  const std::string layerZ = "elevation";

  // borders
  pcl::PointXYZRGB ptmin, ptmax;
  pcl::getMinMax3D(*cloud, ptmin, ptmax);

  // setup gridmap
  gridmap.setFrameId("map");
  gridmap.setGeometry(grid_map::Length(ptmax.x - ptmin.x, ptmax.y - ptmin.y), resolution);

  // insert cloud
  grid_map::Matrix &data = gridmap[layerZ];
  unsigned int npts = cloud->points.size();
  for (unsigned int i = 0; i < npts; i++)
  {
    const pcl::PointXYZRGB &pt = cloud->points[i];
    grid_map::Position position(pt.x, pt.y);
    grid_map::Index index;
    if (gridmap.getIndex(position, index))
    {
      double z = data(index(0), index(1));
      if (std::isnan(z) || z < pt.z)
      {
        data(index(0), index(1)) = pt.z;
      }
    }
  }
  return true;
}

int main(int argc, char *argv[])
{
  // ros
  ros::init(argc, argv, "gridmap_fsc_demo_node");
  ros::NodeHandle nodeHandle("~");

  // params
  double resolution;
  std::string base = ros::package::getPath("recast_demos");
  std::string path;
  nodeHandle.param("resolution", resolution, 0.05);
  nodeHandle.param("path", path, base + std::string("/data/fsc-oil-rig-map-full-1cm-clean.pcd"));

  // gridmap
  grid_map::GridMap gridmap({"elevation"});

  // load PCL file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read .pcd file \n");
    ROS_ERROR("Couldn't read .pcd file \n");
    return -1;
  }
  // chop off top floor
  ROS_INFO("Cropping...");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
  //pcl::PassThrough<pcl::PointXYZRGB> pass;
  //pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (-10.0, 1.0);
  //pass.filter (*cloud_filtered);
  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    const pcl::PointXYZRGB &pt = cloud->points[i];
    if (pt.x >= 0.0 && pt.x <= 40.0 &&
        pt.y >= 0.0 && pt.y <= 80.0 &&
        pt.z >= -10.0 && pt.z <= 1.0)
    {
      cloud_filtered->points.push_back(pt);
    }
  }
  // convert to gridmap
  ROS_INFO("Converting to grid_map...");
  convert(cloud_filtered, gridmap, resolution);
  // fix some issues with the map
  for (grid_map::SpiralIterator iterator(gridmap, grid_map::Position(9.65, 22.3), 0.70); !iterator.isPastEnd(); ++iterator)
  { // close to stairs
    gridmap.at("elevation", *iterator) = -1.28;
  }
  for (grid_map::SpiralIterator iterator(gridmap, grid_map::Position(10.6, 32.1), 0.30); !iterator.isPastEnd(); ++iterator)
  { // close to stairs bottom
    gridmap.at("elevation", *iterator) = -1.38;
  }
  for (grid_map::SpiralIterator iterator(gridmap, grid_map::Position(10, 36.0), 0.80); !iterator.isPastEnd(); ++iterator)
  { // close to slab
    gridmap.at("elevation", *iterator) = -1.40;
  }

  // publish
  ros::Publisher gridMapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("map", 1, true);
  ros::Rate loop_rate(0.25);
  unsigned int count = 0;
  while (ros::ok())
  {
    // set header
    ros::Time time = ros::Time::now();
    gridmap.setTimestamp(time.toNSec());
    gridmap.setFrameId("map");
    // convert
    grid_map_msgs::GridMap mapMessage;
    grid_map::GridMapRosConverter::toMessage(gridmap, mapMessage);
    // publish
    gridMapPublisher.publish(mapMessage);
    ROS_INFO("Published 1 map.");
    // continue
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}

  // Copyright (c) 2019 Martim Brandão martim@robots.ox.ac.uk, Omer Burak Aladag aladagomer@sabanciuniv.edu, Ioannis Havoutis ioannis@robots.ox.ac.uk
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






#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
#include <fstream>

struct Save
{
  Save(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
  {
    std::string base = ros::package::getPath("recast_demos");
    nodeHandle_.param("input_topic", inputTopic_, std::string("/gridmap_fsc_demo_node/map_filtered"));
    nodeHandle_.param("save_path", savePath_, base+std::string("/data/map.obj"));
    nodeHandle_.param("save_path_rotated", savePathRotated_, base+std::string("/data/map_rotated.obj"));
    nodeHandle_.param("save_path_areas", savePathAreas_, base+std::string("/data/map.csv"));
    subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &Save::callback, this);
  }
  void callback(const grid_map_msgs::GridMap& msg)
  {
    // get message
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(msg, gridMap);
    ROS_INFO("received GridMap");
    // convert to pcl polygonmesh
    pcl::PolygonMesh pclMesh;
    std::vector<char> trilabels;
    convert(gridMap, pclMesh, trilabels);
    ROS_INFO("converted to pcl");
    // change coords
    pcl::PolygonMesh pclMeshRotated = pclMesh;
    pcl::PointCloud<pcl::PointXYZ> temp, temp0; // temp array
    //pcl::fromPCLPointCloud2(pclMeshRotated.cloud, temp);
    pcl::fromPCLPointCloud2(pclMeshRotated.cloud, temp0);
    for(int i = 0; i < temp0.size(); i++)
    {
      //temp0.at(i).x = temp.at(i).x; // keep X same | not needed.
      // Update Y and Z
      double tmpy = temp0.at(i).y;
      temp0.at(i).y = temp0.at(i).z;
      temp0.at(i).z = -tmpy;
    }
    pcl::toPCLPointCloud2(temp0, pclMeshRotated.cloud);
    // save to file
    pcl::io::saveOBJFile(savePath_, pclMesh); // pcl::io::savePLYFile(savePath_, pclMesh);
    ROS_INFO("saved OBJ to file");
    pcl::io::saveOBJFile(savePathRotated_, pclMeshRotated); // pcl::io::savePLYFile(savePath_, pclMesh);
    ROS_INFO("saved OBJ (rotated) to file");
    saveAreas(savePathAreas_, trilabels);
    ROS_INFO("saved AREAS to file");
    // test
    pcl::PolygonMesh pclMesh2;
    pcl::io::loadPolygonFileOBJ(savePath_, pclMesh2); // pcl::io::loadPolygonFilePLY(savePath_, pclMesh2);
    pcl::PointCloud<pcl::PointXYZ> verts, verts2;
    pcl::fromPCLPointCloud2(pclMesh.cloud, verts);
    pcl::fromPCLPointCloud2(pclMesh2.cloud, verts2);
    bool ok = (verts.size() == verts2.size());
    for (size_t i = 0; i < verts.size() && ok; i++) {
      if (fabs(verts.at(i).x - verts2.at(i).x) > 1e-2){ ok = false; ROS_ERROR("x = %f vs %f", verts.at(i).x, verts2.at(i).x); }
      if (fabs(verts.at(i).y - verts2.at(i).y) > 1e-2){ ok = false; ROS_ERROR("y = %f vs %f", verts.at(i).y, verts2.at(i).y); }
      if (fabs(verts.at(i).z - verts2.at(i).z) > 1e-2){ ok = false; ROS_ERROR("z = %f vs %f", verts.at(i).z, verts2.at(i).z); }
    }
    std::vector<char> trilabels2;
    loadAreas(savePathAreas_, trilabels2);
    ok = ok && (trilabels.size() == trilabels2.size());
    for (size_t i = 0; i < trilabels.size() && ok; i++) {
      if (trilabels[i] != trilabels2[i]){ ok = false; ROS_ERROR("l = %d vs %d", trilabels[i], trilabels2[i]); }
    }
    // TODO check
    if (ok)
      ROS_INFO("finished");
    else
      ROS_ERROR("save/load test failed\n");
  }
  bool saveAreas(const std::string& path, const std::vector<char>& labels)
  {
    std::ofstream FILE(path, std::ios::out | std::ofstream::binary);
    std::copy(labels.begin(), labels.end(), std::ostreambuf_iterator<char>(FILE));
    FILE.close();
    return true;
  }
  bool loadAreas(const std::string& path, std::vector<char>& labels)
  {
    std::ifstream INFILE(path, std::ios::in | std::ifstream::binary);
    std::istreambuf_iterator<char> eos;
    std::istreambuf_iterator<char> iter(INFILE);
    std::copy(iter, eos, std::back_inserter(labels));
    INFILE.close();
    return true;
  }
  bool convert(const grid_map::GridMap& gridMap, pcl::PolygonMesh& mesh, std::vector<char>& trilabels)
  {
    const std::string layerZ = "elevation";
    const std::string layerCstep = "robot_controller_unfeasibility_steps";
    const std::string layerCtrot = "robot_controller_unfeasibility_trot";
    const float resolution = gridMap.getResolution();
    const grid_map::Size size = gridMap.getSize();

    // aux
    grid_map::Position position;
    grid_map::Index i1,i2,i3;
    bool ok1,ok2,ok3;
    float z1,z2,z3;
    std::vector<float> pts;
    pts.reserve(size(0)*size(1)*9*2);
    trilabels.reserve(size(0)*size(1)*3*2);

    // add cells
    for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
      // location
      gridMap.getPosition(*iterator, position);
      float x = position.x();
      float y = position.y();
      float z = gridMap.at(layerZ, *iterator);
      if (std::isnan(z)) continue;
      // area (controller) type
      float unfeasibilityStep = gridMap.at(layerCstep, *iterator);
      float unfeasibilityTrot = gridMap.at(layerCtrot, *iterator);
      if (std::isnan(unfeasibilityStep)) continue;
      if (std::isnan(unfeasibilityTrot)) continue;
      char label = 0;
      if (unfeasibilityTrot < 1)
        label = 1;
      else if (unfeasibilityStep < 1)
        label = 2;
      else
        continue;
      // neighbor points
      ok1 = gridMap.getIndex(grid_map::Position(x+resolution, y           ), i1);
      ok2 = gridMap.getIndex(grid_map::Position(x           , y+resolution), i2);
      ok3 = gridMap.getIndex(grid_map::Position(x+resolution, y-resolution), i3);
      if (ok1) z1 = gridMap.at(layerZ, i1);
      if (ok2) z2 = gridMap.at(layerZ, i2);
      if (ok3) z3 = gridMap.at(layerZ, i3);
      // triangle up
      if (ok1 && ok2 && !std::isnan(z1) && !std::isnan(z2)) {
        pts.push_back(x);            pts.push_back(y);            pts.push_back(z);   //labels.push_back(label);
        pts.push_back(x+resolution); pts.push_back(y);            pts.push_back(z1);  //labels.push_back(label);
        pts.push_back(x);            pts.push_back(y+resolution); pts.push_back(z2);  //labels.push_back(label);
        trilabels.push_back(label);
      }
      // triangle down
      if (ok1 && ok3 && !std::isnan(z1) && !std::isnan(z3)) {
        pts.push_back(x);            pts.push_back(y);            pts.push_back(z);   //labels.push_back(label);
        pts.push_back(x+resolution); pts.push_back(y-resolution); pts.push_back(z3);  //labels.push_back(label);
        pts.push_back(x+resolution); pts.push_back(y);            pts.push_back(z1);  //labels.push_back(label);
        trilabels.push_back(label);
      }
    }

    // convert to pcl mesh
    int npts = pts.size()/3;
    int ntri = npts/3;
    mesh.polygons.resize(ntri);
    for (int i = 0; i < ntri; i++) {
      mesh.polygons[i].vertices.resize(3);
      mesh.polygons[i].vertices[0] = i*3+0;
      mesh.polygons[i].vertices[1] = i*3+1;
      mesh.polygons[i].vertices[2] = i*3+2;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
    cloud->points.resize(pts.size());
    for (int i = 0; i < npts; i++) {
      cloud->points[i].x = pts[i*3+0];
      cloud->points[i].y = pts[i*3+1];
      cloud->points[i].z = pts[i*3+2];
    }
    pcl::toPCLPointCloud2 (*cloud, mesh.cloud);

    return true;
  }
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
  std::string inputTopic_;
  std::string savePath_;
  std::string savePathRotated_;
  std::string savePathAreas_;
  
};


int main (int argc, char* argv[])
{
  ros::init(argc, argv, "gridmap_save_node");
  ros::NodeHandle nodeHandle("~");
  Save save(nodeHandle);
  ros::spin();
  return 0;
}


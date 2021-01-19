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

#include "Sample.h"
#include "InputGeom.h"
#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <memory>
#include <string>

namespace recast_ros
{

class RecastPlanner
{
public:
  RecastPlanner();
  //Calls necessary Recast functions and builds NavMesh
  bool build(const pcl::PolygonMesh &pclMesh, const std::vector<char> &areaTypes, const int &maxNodeSize);
  //Loads map and areatypes from respective files and calls RecastPlanner::build(...)
  bool loadAndBuild(const std::string &mapFile, const std::string &areaFile, const int &maxNodeSize);
  //Path query for RecastPlanner, uses findPath / findStraightPath and returns resulting optimal path
  bool query(const pcl::PointXYZ &start, const pcl::PointXYZ &end, std::vector<pcl::PointXYZ> &path, const std::vector<float> &areaCostList, const int &areaTypeCount, const int &noPolygons);
  //Interactive tool to project 3D goal point to 2D for test_planning_service_interactive
  bool getProjection(const pcl::PointXYZ &point, pcl::PointXYZ &proj, unsigned char &areaType);
  bool getProjection(const pcl::PointXYZ &point, pcl::PointXYZ &proj, unsigned char &areaType, pcl::PointXYZ &polyCenter);
  std::shared_ptr<Sample> getMySample() { return sample; }
  //Add RecastObstacles to NavMesh, updates the tiles/NavMesh
  bool addRecastObstacle(const float *pos, const float &radi, const float &height);
  //Remove specific RecastObstacle from NavMesh, updates the tiles/NavMesh
  void removeRecastObstacle(const float *pos);
  void clearAllRecastObstacles();
  void update();
  bool drawRecastGraph(std::vector<float> &graphNodes);
  bool drawRecastGraph(std::vector<float> &graphNodes, std::vector<float> &graphPortals, std::vector<unsigned char> &areaTypes, std::vector<int> &idxTriangles, std::vector<int> &numTriangles);
  //Converts dtNavMesh map to pcl::PolygonMesh
  bool getNavMesh(pcl::PolygonMesh::Ptr &pclmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &pclcloud, std::vector<Eigen::Vector3d> &lineList, std::vector<unsigned char> &areaList, int &noPolygons) const;
  bool getNavMesh(pcl::PolygonMesh::Ptr &pclmesh) const;
  bool getNavMesh(std::vector<Eigen::Vector3d> &lineList) const;
  BuildSettings stg;

private:
  BuildContext ctx;
  std::shared_ptr<Sample> sample;
  std::shared_ptr<InputGeom> geom;
  bool needToRotateMesh; // Recast axes and ROS axes are different, thus input map may have to be rotated.
};

//Optional :: AreaTypes loaded from .dat file
bool loadAreas(const std::string &path, std::vector<char> &labels);

template <class T>
inline T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

} // namespace recast_ros

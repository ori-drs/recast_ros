#include "Sample.h"
#include "InputGeom.h"
#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/shared_ptr.hpp>
#include <string>

namespace recast_ros
{

class RecastPlanner
{
public:
  RecastPlanner();
  //Calls necessary Recast functions and builds NavMesh
  bool build(const pcl::PolygonMesh &pclMesh, const std::vector<char> &areaTypes, const int & maxNodeSize);
  //Loads map and areatypes from respective files and calls RecastPlanner::build(...)
  bool loadAndBuild(const std::string &mapFile, const std::string &areaFile, const int & maxNodeSize);
  //Path query for RecastPlanner, uses findPath / findStraightPath and returns resulting optimal path
  bool query(const pcl::PointXYZ &start, const pcl::PointXYZ &end, std::vector<pcl::PointXYZ> &path, const std::vector<float> &areaCostList, const int &areaTypeCount, const int & noPolygons);
  //Interactive tool to project 3D goal point to 2D for test_planning_service_interactive
  bool getProjection(const pcl::PointXYZ &point, pcl::PointXYZ &proj, unsigned char &areaType);
  boost::shared_ptr<Sample> getMySample() { return sample; }
  //Add RecastObstacles to NavMesh, updates the tiles/NavMesh
  bool addRecastObstacle(const float *pos, const float &radi, const float &height);
  void clearAllRecastObstacles();
  void update();
  //Converts dtNavMesh map to pcl::PolygonMesh
  bool getNavMesh(pcl::PolygonMesh::Ptr &pclmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &pclcloud, std::vector<Eigen::Vector3d> &lineList, std::vector<unsigned char> &areaList, int &noPolygons) const;
  bool getNavMesh(pcl::PolygonMesh::Ptr &pclmesh) const;
  bool getNavMesh(std::vector<Eigen::Vector3d> &lineList) const;
  BuildSettings stg;

private:
  BuildContext ctx;
  boost::shared_ptr<Sample> sample;
  boost::shared_ptr<InputGeom> geom;
  bool needToRotateMesh; // Recast axes and ROS axes are different, thus input map may have to be rotated.
};

//Optional :: AreaTypes loaded from .dat file
bool loadAreas(const std::string &path, std::vector<char> &labels);

template <class T>
inline T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

}

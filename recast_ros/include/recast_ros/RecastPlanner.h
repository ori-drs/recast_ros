#include "Sample.h"
#include "InputGeom.h"
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
  bool build(const pcl::PolygonMesh &pclMesh, const std::vector<char> &areaTypes);
  bool loadAndBuild(const std::string &mapFile, const std::string &areaFile);
  bool query(const pcl::PointXYZ &start, const pcl::PointXYZ &end, std::vector<pcl::PointXYZ> &path, const std::vector<float> &areaCostList, const int &areaTypeCount, const int & noPolygons);
  bool getProjection(const pcl::PointXYZ &point, pcl::PointXYZ &proj, unsigned char &areaType);
  boost::shared_ptr<Sample> getMySample() { return sample; }
  bool addRecastObstacle(const float *pos, const float &radi, const float &height);
  void clearAllRecastObstacles();
  void update();
  bool getNavMesh(pcl::PolygonMesh::Ptr &pclmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &pclcloud, std::vector<Eigen::Vector3d> &lineList, std::vector<unsigned char> &areaList, int &noPolygons) const;
  bool getNavMesh(pcl::PolygonMesh::Ptr &pclmesh) const;
  bool getNavMesh(std::vector<Eigen::Vector3d> &lineList) const;
  //  float getDtAreaCost(const int &index);
  // void setDtAreaCost(const int &index, const int &cost);
  BuildSettings stg;

private:
  BuildContext ctx;
  boost::shared_ptr<Sample> sample;
  boost::shared_ptr<InputGeom> geom;
  bool needToRotateMesh;
};

bool loadAreas(const std::string &path, std::vector<char> &labels);

template <class T>
inline T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

} // namespace recast_ros

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

#include "recast_ros/RecastPlanner.h"
#include "recast_ros/MySampleObstacles.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
//#include "NavMeshTesterTool.h"
#include "DetourCommon.h"
#include <Eigen/Dense>
#include <pcl/io/vtk_lib_io.h>

using namespace recast_ros;

RecastPlanner::RecastPlanner() : needToRotateMesh(true)
{
  stg.cellSize = 0.1f;
  stg.cellHeight = 0.1f;
  stg.agentHeight = 0.7f;
  stg.agentRadius = 0.2f;
  stg.agentMaxClimb = 0.41f;
  stg.agentMaxSlope = 60.0f;
  stg.regionMinSize = 8;
  stg.regionMergeSize = 20;
  stg.edgeMaxLen = 12.0f;
  stg.edgeMaxError = 1.3f;
  stg.vertsPerPoly = 6.0f;
  stg.detailSampleDist = 6.0f;
  stg.detailSampleMaxError = 1.0f;
  stg.partitionType = SAMPLE_PARTITION_WATERSHED;
}

bool RecastPlanner::build(const pcl::PolygonMesh &pclMesh, const std::vector<char> &areaTypes, const int &maxNodeSize)
{
  // Build navmesh
  geom = boost::shared_ptr<InputGeom>(new InputGeom());
  if (!geom->load(&ctx, pclMesh, needToRotateMesh))
    return false;
  //Recast settings are set
  geom->setBuildSettings(stg);
  sample = boost::shared_ptr<Sample>(new MySampleObstacles());
  sample->setContext(&ctx);
  sample->handleMeshChanged(geom.get());
  sample->handleSettings(maxNodeSize);
  sample->handleBuild(areaTypes, maxNodeSize);

  if (!sample)
    return false;

  return true;
}

bool RecastPlanner::loadAndBuild(const std::string &mapFile, const std::string &areaFile, const int &maxNodeSize)
{
  // Load mesh file
  pcl::PolygonMesh pclMesh;
  std::vector<char> areaTypes;
  if (!pcl::io::loadPolygonFile(mapFile, pclMesh))
    return false;
  if (!loadAreas(areaFile, areaTypes))
    return false;
  return build(pclMesh, areaTypes, maxNodeSize);
}

//High Level Path Planner
bool RecastPlanner::query(const pcl::PointXYZ &start, const pcl::PointXYZ &end, std::vector<pcl::PointXYZ> &path, const std::vector<float> &areaCostList, const int &areaTypeCount, const int &noPolygons)
{
  if (!sample)
    return false;
  dtNavMesh *navmesh = sample->getNavMesh();
  dtNavMeshQuery *navquery = sample->getNavMeshQuery();
  if (!navmesh || !navquery)
    return false;

  //MAX_POLYS is buffer size for path length. Longer paths/Bigger maps requires bigger buffer size. Default = 256
  static const int MAX_POLYS = std::max(256, noPolygons);
  dtPolyRef polys[MAX_POLYS];
  float straight[MAX_POLYS * 3];
  const float polyPickExt[3] = {2, 4, 2};
  int npolys = 0;
  int nstraight = 0;

  dtQueryFilter filter;
  filter.setIncludeFlags(0x3);
  filter.setExcludeFlags(0x0);

  //Area Types starts from 1, thus areatype0 has no cost
  filter.setAreaCost(0, 0);

  for (int index = 1; index < areaTypeCount; index++)
  {
    filter.setAreaCost(index, areaCostList[index]);
  }

  // Convert ROS Axes | Recast Axes
  float spos[3];
  float epos[3];
  float nspos[3];
  float nepos[3];
  if (needToRotateMesh)
  {
    spos[0] = start.x;
    spos[1] = start.z;
    spos[2] = -start.y;
    epos[0] = end.x;
    epos[1] = end.z;
    epos[2] = -end.y;
  }
  else
  {
    spos[0] = start.x;
    spos[1] = start.y;
    spos[2] = start.z;
    epos[0] = end.x;
    epos[1] = end.y;
    epos[2] = end.z;
  }

  // Find start points
  dtPolyRef startRef, endRef;
  navquery->findNearestPoly(spos, polyPickExt, &filter, &startRef, nspos);
  navquery->findNearestPoly(epos, polyPickExt, &filter, &endRef, nepos);
  if (!startRef || !endRef)
    return false;

  // Find path
  bool foundFullPath = false;
  dtStatus myRes = navquery->findPath(startRef, endRef, spos, epos, &filter, polys, &npolys, MAX_POLYS);
  if (myRes == DT_SUCCESS)
    foundFullPath = true;

  // Find straight path
  if (npolys)
    navquery->findStraightPath(spos, epos, polys, npolys, straight, 0, 0, &nstraight, MAX_POLYS);
  else
    return false;

  // Convert ROS Axes | Recast Axes
  path.resize(nstraight);
  if (needToRotateMesh)
  {
    for (int i = 0; i < nstraight; i++)
    {
      path[i].x = straight[i * 3 + 0];  // in Recast x points front
      path[i].y = -straight[i * 3 + 2]; // in Recast z points right
      path[i].z = straight[i * 3 + 1];  // in Recast y points up
    }
  }
  else
  {
    for (int i = 0; i < nstraight; i++)
    {
      path[i].x = straight[i * 3 + 0];
      path[i].y = straight[i * 3 + 1];
      path[i].z = straight[i * 3 + 2];
    }
  }
  return foundFullPath;
}
//Assume cylindrical obstacle
//WARNING ! Each operation regarding to Obstacles requires a map update afterwards
//Removes obstacle from the clicked position pos
void RecastPlanner::removeRecastObstacle(const float *pos)
{
  float hitTime;
  float rayStart[3], rayEnd[3];
  bool hit = geom->raycastMesh(rayStart, rayEnd, hitTime);
  // Convert ROS Axes | Recast Axes
  float x[3] = {pos[0], pos[2], -pos[1]};

  sample->removeTempObstacle(rayStart, x);
}

bool RecastPlanner::addRecastObstacle(const float *pos, const float &radi, const float &height)
{
  // Convert ROS Axes | Recast Axes
  float x[3] = {pos[0], pos[2], -pos[1]};

  dtStatus res = sample->addTempObstacle(x, radi, height);

  if (res == DT_SUCCESS)
    return true;
  else
    return false;
}
void RecastPlanner::clearAllRecastObstacles()
{
  sample->clearAllTempObstacles();
  this->update();
}

void RecastPlanner::update()
{
  // Update sample simulation.
  float deltaTime = 0.01;
  sample->handleUpdate(deltaTime);
}
//test_planning_service_interactive function for 2D Nav Goal functionality
bool RecastPlanner::getProjection(const pcl::PointXYZ &point, pcl::PointXYZ &proj, unsigned char &areaType)
{
  if (!sample)
    return false;
  dtNavMesh *navmesh = sample->getNavMesh();
  dtNavMeshQuery *navquery = sample->getNavMeshQuery();
  if (!navmesh || !navquery)
    return false;

  const float polyPickExt[3] = {2, 4, 2};
  dtQueryFilter filter;
  filter.setIncludeFlags(0x3);
  filter.setExcludeFlags(0x0);

  // Convert
  float spos[3];
  float nspos[3];
  if (needToRotateMesh)
  {
    spos[0] = point.x;
    spos[1] = point.z;
    spos[2] = -point.y;
  }
  else
  {
    spos[0] = point.x;
    spos[1] = point.y;
    spos[2] = point.z;
  }

  // Find polygon
  dtPolyRef ref;
  navquery->findNearestPoly(spos, polyPickExt, &filter, &ref, nspos);
  if (!ref)
    return false;

  // Get projected point
  if (needToRotateMesh)
  {
    proj.x = nspos[0];  // in Recast x points front
    proj.y = -nspos[2]; // in Recast z points right
    proj.z = nspos[1];  // in Recast y points up
  }
  else
  {
    proj.x = nspos[0];
    proj.y = nspos[1];
    proj.z = nspos[2];
  }

  // Get area type
  if (navmesh->getPolyArea(ref, &areaType) != DT_SUCCESS)
    return false;

  return true;
}

//Same function from NavMeshTesterTool in original recastnavigation
static void getPolyCenter(const dtNavMesh *navMesh, dtPolyRef ref, float *center)
{
  center[0] = 0;
  center[1] = 0;
  center[2] = 0;

  const dtMeshTile *tile = 0;
  const dtPoly *poly = 0;
  dtStatus status = navMesh->getTileAndPolyByRef(ref, &tile, &poly);
  if (dtStatusFailed(status))
    return;

  for (int i = 0; i < (int)poly->vertCount; ++i)
  {
    const float *v = &tile->verts[poly->verts[i] * 3];
    center[0] += v[0];
    center[1] += v[1];
    center[2] += v[2];
  }
  const float s = 1.0f / poly->vertCount;
  center[0] *= s;
  center[1] *= s;
  center[2] *= s;
}

//Same function from NavMeshTesterTool in original recastnavigation
static void getPolyCenter(const dtNavMesh *navMesh, const dtPoly *poly, const dtMeshTile *tile, float *center)
{
  center[0] = 0;
  center[1] = 0;
  center[2] = 0;

  for (int i = 0; i < (int)poly->vertCount; ++i)
  {
    const float *v = &tile->verts[poly->verts[i] * 3];
    center[0] += v[0];
    center[1] += v[1];
    center[2] += v[2];
  }
  const float s = 1.0f / poly->vertCount;
  center[0] *= s;
  center[1] *= s;
  center[2] *= s;
}

//Same function from DetourNavMeshQuery in original recastnavigation... Returns portal points between two polygons.
static bool getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
                            dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
                            float* left, float* right)
{
  // Find the link that points to the 'to' polygon.
  const dtLink* link = 0;
  for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next) {
    if (fromTile->links[i].ref == to) {
      link = &fromTile->links[i];
      break;
    }
  }
  if (!link)
    return false;

  // Handle off-mesh connections.
  if (fromPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) {
    // Find link that points to first vertex.
    for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next) {
      if (fromTile->links[i].ref == to) {
        const int v = fromTile->links[i].edge;
        dtVcopy(left, &fromTile->verts[fromPoly->verts[v]*3]);
        dtVcopy(right, &fromTile->verts[fromPoly->verts[v]*3]);
        return true;
      }
    }
    return false;
  }

  if (toPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) {
    for (unsigned int i = toPoly->firstLink; i != DT_NULL_LINK; i = toTile->links[i].next) {
      if (toTile->links[i].ref == from) {
        const int v = toTile->links[i].edge;
        dtVcopy(left, &toTile->verts[toPoly->verts[v]*3]);
        dtVcopy(right, &toTile->verts[toPoly->verts[v]*3]);
        return true;
      }
    }
    return false;
  }

  // Find portal vertices.
  const int v0 = fromPoly->verts[link->edge];
  const int v1 = fromPoly->verts[(link->edge+1) % (int)fromPoly->vertCount];
  dtVcopy(left, &fromTile->verts[v0*3]);
  dtVcopy(right, &fromTile->verts[v1*3]);

  // If the link is at tile boundary, dtClamp the vertices to
  // the link width.
  if (link->side != 0xff) {
    // Unpack portal limits.
    if (link->bmin != 0 || link->bmax != 255) {
      const float s = 1.0f/255.0f;
      const float tmin = link->bmin*s;
      const float tmax = link->bmax*s;
      dtVlerp(left, &fromTile->verts[v0*3], &fromTile->verts[v1*3], tmin);
      dtVlerp(right, &fromTile->verts[v0*3], &fromTile->verts[v1*3], tmax);
    }
  }

  return true;
}

//Same function from DetourNavMeshQuery in original recastnavigation
static bool getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
										 dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
										 float* mid)
{
	float left[3], right[3];
	if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
		return false;
	mid[0] = (left[0]+right[0])*0.5f;
	mid[1] = (left[1]+right[1])*0.5f;
	mid[2] = (left[2]+right[2])*0.5f;
	return true;
}

bool RecastPlanner::getProjection(const pcl::PointXYZ &point, pcl::PointXYZ &proj, unsigned char &areaType, pcl::PointXYZ &polyCenter)
{
  if (!sample)
    return false;
  dtNavMesh *navmesh = sample->getNavMesh();
  dtNavMeshQuery *navquery = sample->getNavMeshQuery();
  if (!navmesh || !navquery)
    return false;

  const float polyPickExt[3] = {2, 4, 2};
  dtQueryFilter filter;
  filter.setIncludeFlags(0x3);
  filter.setExcludeFlags(0x0);

  // Convert
  float spos[3];
  float nspos[3];
  if (needToRotateMesh)
  {
    spos[0] = point.x;
    spos[1] = point.z;
    spos[2] = -point.y;
  }
  else
  {
    spos[0] = point.x;
    spos[1] = point.y;
    spos[2] = point.z;
  }

  // Find polygon
  dtPolyRef ref;
  navquery->findNearestPoly(spos, polyPickExt, &filter, &ref, nspos);
  if (!ref)
    return false;

  // Get projected point
  if (needToRotateMesh)
  {
    proj.x = nspos[0];  // in Recast x points front
    proj.y = -nspos[2]; // in Recast z points right
    proj.z = nspos[1];  // in Recast y points up
  }
  else
  {
    proj.x = nspos[0];
    proj.y = nspos[1];
    proj.z = nspos[2];
  }

  // Get area type
  if (navmesh->getPolyArea(ref, &areaType) != DT_SUCCESS)
    return false;

  // Get projected polygon
  float pppos[3];
  getPolyCenter(navmesh, ref, pppos);
  if (needToRotateMesh)
  {
    polyCenter.x = pppos[0];  // in Recast x points front
    polyCenter.y = -pppos[2]; // in Recast z points right
    polyCenter.z = pppos[1];  // in Recast y points up
  }
  else
  {
    polyCenter.x = pppos[0];
    polyCenter.y = pppos[1];
    polyCenter.z = pppos[2];
  }

  return true;
}

//Builds non-directional graph of reachable polygons from reference_point in a circular region with radius searchRadius
bool RecastPlanner::drawRecastGraph(std::vector<float> &graphNodes)
{
  if (!sample)
  {
    ROS_ERROR("SampleObj FAILED");
    return false;
  }
  const dtNavMesh *mesh = sample->getNavMesh();
  if (!mesh)
  {
    ROS_ERROR("dtNavMesh FAILED");
    return false;
  }

  // go through all tiles

  if (mesh->getMaxTiles() < 1)
  {
    ROS_ERROR("Max tiles are 0");
    return false;
  }
  for (int i = 0; i < mesh->getMaxTiles(); ++i)
  {
    const dtMeshTile *tile = mesh->getTile(i);
    if (!tile->header)
      continue;
    for (int i = 0; i < tile->header->polyCount; ++i)
    {
      dtPoly *p = &tile->polys[i];
      dtPolyRef pRef = tile->links[i].ref;
      if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) // Skip off-mesh links.
        continue;
      for (unsigned int j = p->firstLink; j != DT_NULL_LINK; j = tile->links[j].next)
      {
        dtPolyRef neighbourRef = tile->links[j].ref;
        const dtMeshTile *neighbourTile = 0;
        const dtPoly *neighbourPoly = 0;
        mesh->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
        float c0[3], c1[3];

        getPolyCenter(mesh, neighbourRef, c0);
        getPolyCenter(mesh, p, tile, c1);

        graphNodes.push_back(c0[0]);
        graphNodes.push_back(-c0[2]);
        graphNodes.push_back(c0[1]);

        graphNodes.push_back(c1[0]);
        graphNodes.push_back(-c1[2]);
        graphNodes.push_back(c1[1]);
      }
    }
  }

  return true;
}

bool RecastPlanner::drawRecastGraph(std::vector<float> &graphNodes, std::vector<float> &graphPortals, std::vector<unsigned char> &areaTypes, std::vector<int> &idxTriangles, std::vector<int> &numTriangles)
{
  if (!sample)
  {
    ROS_ERROR("SampleObj FAILED");
    return false;
  }
  const dtNavMesh *mesh = sample->getNavMesh();
  if (!mesh)
  {
    ROS_ERROR("dtNavMesh FAILED");
    return false;
  }
  if (mesh->getMaxTiles() < 1)
  {
    ROS_ERROR("Max tiles are 0");
    return false;
  }

  // store idx and num triangles for each node
  int totalTriangles = 0;
  std::map<const dtPoly*, std::pair<int,int> > polyref2idxnum;
  for (int t = 0; t < mesh->getMaxTiles(); ++t)
  {
    const dtMeshTile *tile = mesh->getTile(t);
    if (!tile->header)
      continue;
    for (int i = 0; i < tile->header->polyCount; ++i)
    {
      dtPoly *p = &tile->polys[i];
      dtPolyRef pRef = tile->links[i].ref;
      if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) // Skip off-mesh links.
        continue;
      polyref2idxnum[p] = std::pair<int,int>(totalTriangles*3, tile->detailMeshes[i].triCount);
      totalTriangles += tile->detailMeshes[i].triCount;
    }
  }

  int count0 = 0;
  int count1 = 0;

  // go through all tiles
  for (int t = 0; t < mesh->getMaxTiles(); ++t)
  {
    const dtMeshTile *tile = mesh->getTile(t);
    if (!tile->header)
      continue;
    for (int i = 0; i < tile->header->polyCount; ++i)
    {
      dtPoly *p = &tile->polys[i];
      dtPolyRef pRef = tile->links[i].ref;
      if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) // Skip off-mesh links.
        continue;
      for (unsigned int j = p->firstLink; j != DT_NULL_LINK; j = tile->links[j].next)
      {
        dtPolyRef neighbourRef = tile->links[j].ref;
        const dtMeshTile *neighbourTile = 0;
        const dtPoly *neighbourPoly = 0;
        mesh->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
        float c0[3], c1[3];

        getPolyCenter(mesh, neighbourRef, c0);
        getPolyCenter(mesh, p, tile, c1);

        graphNodes.push_back(c0[0]);
        graphNodes.push_back(-c0[2]);
        graphNodes.push_back(c0[1]);

        graphNodes.push_back(c1[0]);
        graphNodes.push_back(-c1[2]);
        graphNodes.push_back(c1[1]);

        // triangles
        const std::pair<int,int> idxnum0 = polyref2idxnum[neighbourPoly];
        const std::pair<int,int> idxnum1 = polyref2idxnum[p];
        idxTriangles.push_back(idxnum0.first);
        idxTriangles.push_back(idxnum1.first);
        numTriangles.push_back(idxnum0.second);
        numTriangles.push_back(idxnum1.second);

        // portal point between two nodes. TODO: do we need both directions here?
        float mid[3];
        getEdgeMidPoint(pRef, p, tile, neighbourRef, neighbourPoly, neighbourTile, mid);
        graphPortals.push_back(mid[0]);
        graphPortals.push_back(-mid[2]);
        graphPortals.push_back(mid[1]);

        // area types
        unsigned char a0, a1;
        if (mesh->getPolyArea(neighbourRef, &a0) != DT_SUCCESS) return false;
        if (mesh->getPolyArea(pRef, &a1) != DT_SUCCESS) return false;
        areaTypes.push_back(a0);
        areaTypes.push_back(a1);
      }
    }
  }

  return true;
}

//Converts dtNavMesh => pcl::PolygonMesh
bool RecastPlanner::getNavMesh(pcl::PolygonMesh::Ptr &pclmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr &pclcloud, std::vector<Eigen::Vector3d> &lineList, std::vector<unsigned char> &areaList, int &noPolygons) const
{
  if (!sample)
  {
    ROS_ERROR("SampleObj FAILED");
    return false;
  }
  const dtNavMesh *mesh = sample->getNavMesh();
  if (!mesh)
  {
    ROS_ERROR("dtNavMesh FAILED");
    return false;
  }

  pclmesh.reset(new pcl::PolygonMesh());
  pclcloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

  // go through all tiles
  pclcloud->points.reserve(mesh->getMaxTiles() * 10 * 3);

  if (mesh->getMaxTiles() < 1)
  {
    ROS_ERROR("Max tiles are 0");
    return false;
  }
  for (int t = 0; t < mesh->getMaxTiles(); ++t)
  {
    const dtMeshTile *tile = mesh->getTile(t);
    if (!tile->header)
      continue;
    dtPolyRef base = mesh->getPolyRefBase(tile);
    int tileNum = mesh->decodePolyIdTile(base);
    for (int i = 0; i < tile->header->polyCount; ++i)
    {
      const dtPoly *p = &tile->polys[i];
      if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) // Skip off-mesh links.
        continue;
      const dtPolyDetail *pd = &tile->detailMeshes[i];
      // go through all triangles
      for (int j = 0; j < pd->triCount; ++j)
      {
        const unsigned char *t = &tile->detailTris[(pd->triBase + j) * 4];
        for (int k = 0; k < 3; ++k)
        {
          const float *verts;
          if (t[k] < p->vertCount)
            verts = &tile->verts[p->verts[t[k]] * 3];
          else
            verts = &tile->detailVerts[(pd->vertBase + t[k] - p->vertCount) * 3];
          // add vertex to pcl cloud
          pcl::PointXYZ vert;
          vert.x = verts[0];
          vert.y = -verts[2];
          vert.z = verts[1];
          pclcloud->points.push_back(vert);
          areaList.push_back(p->getArea());
        }
      }
    }
  }

  // build pcl mesh
  pcl::toPCLPointCloud2(*pclcloud, pclmesh->cloud);
  int ntri = pclcloud->points.size() / 3;
  pclmesh->polygons.resize(ntri);
  noPolygons = ntri;

  for (int i = 0; i < ntri; i++)
  {
    pclmesh->polygons[i].vertices.resize(3);
    pclmesh->polygons[i].vertices[0] = i * 3 + 0;
    pclmesh->polygons[i].vertices[1] = i * 3 + 1;
    pclmesh->polygons[i].vertices[2] = i * 3 + 2;
  }

  // line list for triangles
  lineList.reserve(ntri * 6);
  for (int i = 0; i < ntri; i++)
  {
    const pcl::PointXYZ &p1 = pclcloud->points[pclmesh->polygons[i].vertices[0]];
    const pcl::PointXYZ &p2 = pclcloud->points[pclmesh->polygons[i].vertices[1]];
    const pcl::PointXYZ &p3 = pclcloud->points[pclmesh->polygons[i].vertices[2]];
    Eigen::Vector3d v1(p1.x, p1.y, p1.z);
    Eigen::Vector3d v2(p2.x, p2.y, p2.z);
    Eigen::Vector3d v3(p3.x, p3.y, p3.z);
    lineList.push_back(v1);
    lineList.push_back(v2);
    lineList.push_back(v2);
    lineList.push_back(v3);
    lineList.push_back(v3);
    lineList.push_back(v1);
  }
  return true;
}

bool RecastPlanner::getNavMesh(pcl::PolygonMesh::Ptr &pclmesh) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud;
  std::vector<Eigen::Vector3d> lineList;
  std::vector<unsigned char> areaList;
  int numPoly = 0;
  return getNavMesh(pclmesh, pclcloud, lineList, areaList, numPoly);
}

bool RecastPlanner::getNavMesh(std::vector<Eigen::Vector3d> &lineList) const
{
  pcl::PolygonMesh::Ptr pclmesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud;
  std::vector<unsigned char> areaList;
  int numPoly = 0;
  return getNavMesh(pclmesh, pclcloud, lineList, areaList, numPoly);
}

bool recast_ros::loadAreas(const std::string &path, std::vector<char> &labels)
{
  // load file with per-triangle area types
  std::ifstream INFILE(path, std::ios::in | std::ifstream::binary);
  std::istreambuf_iterator<char> eos;
  std::istreambuf_iterator<char> iter(INFILE);
  std::copy(iter, eos, std::back_inserter(labels));
  return labels.size() > 0;
}

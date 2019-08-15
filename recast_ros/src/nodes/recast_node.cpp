#include "recast_ros/RecastPlanner.h"
#include "recast_ros/recast_nodeConfig.h"
#include "recast_ros/RecastProjectSrv.h"
#include "recast_ros/RecastPathSrv.h"
#include "recast_ros/AddObstacleSrv.h"
#include "recast_ros/RecastPathMsg.h"
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
#include <fstream>

struct RecastNode
{
  RecastNode(ros::NodeHandle &nodeHandle)
      : nodeHandle_(nodeHandle), noAreaTypes_(20), areaCostList_(noAreaTypes_), colourList_(noAreaTypes_), loopRate_(100.0)
  {
    // ros params
    std::string base = ros::package::getPath("recast_demos");
    std::string temp = "TERRAIN_TYPE", temp1 = "";
    nodeHandle_.param("path", path_, base + std::string("/data/map.obj"));
    nodeHandle_.param("path_areas", pathAreas_, base + std::string("/data/map.dat"));
    nodeHandle_.param("start_x", startX_, 0.0);
    nodeHandle_.param("start_y", startY_, 0.0);
    nodeHandle_.param("start_z", startZ_, 0.0);
    nodeHandle_.param("goal_x", goalX_, 0.0);
    nodeHandle_.param("goal_y", goalY_, 0.0);
    nodeHandle_.param("goal_z", goalZ_, 0.0);
    nodeHandle_.param("cell_size", cellSize_, 0.1f);
    nodeHandle_.param("cell_height", cellHeight_, 0.1f);
    nodeHandle_.param("agent_height", agentHeight_, 0.7f);
    nodeHandle_.param("agent_radius", agentRadius_, 0.2f);
    nodeHandle_.param("agent_max_climb", agentMaxClimb_, 0.41f);
    nodeHandle_.param("agent_max_slope", agentMaxSlope_, 60.0f);

    // ros publishers
    NavMeshPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("navigation_mesh", 1);
    OriginalMeshPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("original_mesh", 1);
    RecastPathPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("recast_path_lines", 1);
    RecastObstaclePub_ = nodeHandle_.advertise<visualization_msgs::Marker>("recast_obstacles", 1);

    // create colour list for area types
    colourList_ = {
        UIntToColor(0xFFFF6800), //Vivid Orange
        UIntToColor(0xFFA6BDD7), //Very Light Blue
        UIntToColor(0xFFC10020), //Vivid Red
        UIntToColor(0xFFCEA262), //Grayish Yellow
        UIntToColor(0xFF817066), //Medium Gray
        UIntToColor(0xFFFFB300), //Vivid Yellow
        UIntToColor(0xFF803E75), //Strong Purple

        //The following will not be good for people with defective color vision
        UIntToColor(0xFF93AA00), //Vivid Yellowish Green
        UIntToColor(0xFF007D34), //Vivid Green
        UIntToColor(0xFFF6768E), //Strong Purplish Pink
        UIntToColor(0xFF00538A), //Strong Blue
        UIntToColor(0xFFFF7A5C), //Strong Yellowish Pink
        UIntToColor(0xFF53377A), //Strong Violet
        UIntToColor(0xFFFF8E00), //Vivid Orange Yellow
        UIntToColor(0xFFB32851), //Strong Purplish Red
        UIntToColor(0xFFF4C800), //Vivid Greenish Yellow
        UIntToColor(0xFF7F180D), //Strong Reddish Brown
        UIntToColor(0xFF593315), //Deep Yellowish Brown
        UIntToColor(0xFFF13A13), //Vivid Reddish Orange
        UIntToColor(0xFF232C16), //Dark Olive Green
    };

    //ROS_INFO("%f %f %f", colourList_[1].r, colourList_[1].g, colourList_[1].b);

    // create service (server & client)
    servicePlan_ = nodeHandle_.advertiseService("plan_path", &RecastNode::findPathService, this);
    serviceProject_ = nodeHandle_.advertiseService("project_point", &RecastNode::projectPointService, this);
    serviceAddObstacle_ = nodeHandle_.advertiseService("add_obstacle", &RecastNode::addObstacleService, this);

    for (size_t i = 1; i < noAreaTypes_; i++)
    {
      temp1 = temp + boost::to_string(i) + "_COST";
      nodeHandle_.param(temp1, areaCostList_[i], 1.0f);
    }
    // recast settings
    recast_.stg.cellSize = cellSize_;
    recast_.stg.cellHeight = cellHeight_;
    recast_.stg.agentHeight = agentHeight_;
    recast_.stg.agentRadius = agentRadius_;
    recast_.stg.agentMaxClimb = agentMaxClimb_;
    recast_.stg.agentMaxSlope = agentMaxSlope_;
  }

  std_msgs::ColorRGBA UIntToColor(uint32_t color)
  {
    std_msgs::ColorRGBA c;
    c.a = ((double)((uint8_t)(color >> 24))) / 255.0;
    c.r = ((double)((uint8_t)(color >> 16))) / 255.0;
    c.g = ((double)((uint8_t)(color >> 8))) / 255.0;
    c.b = ((double)((uint8_t)(color >> 0))) / 255.0;
    return c;
  }

  void setVisualParameters(visualization_msgs::Marker &v, const int &markerType, const std::string &nameSpace, const int &id) // Constructs a marker of Triangle List
  {
    v.header.frame_id = "map";
    v.type = markerType;
    v.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    v.ns = nameSpace;
    v.id = id;

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
    if (v.type == visualization_msgs::Marker::LINE_LIST)
    {
      v.scale.x = 0.03;
      v.scale.y = 0.0;
      v.scale.z = 0.0;
    }
    else
    {
      v.scale.x = 1.0;
      v.scale.y = 1.0;
      v.scale.z = 1.0;
    }

    // Set the color -- be sure to set alpha to something non-zero!
    v.color.r = 0.0f;
    v.color.g = 0.0f;
    v.color.b = 1.0f;
    v.color.a = 0.6;

    v.lifetime = ros::Duration();
  }
  void visualizeObstacle(visualization_msgs::Marker &v, const float *pos, const int &id) // Constructs a marker of Triangle List
  {
    v.header.frame_id = "map";
    v.type = visualization_msgs::Marker::CYLINDER;
    v.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    v.ns = "Obstacles";
    v.id = id;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    v.action = visualization_msgs::Marker::ADD;
    v.pose.position.x = pos[0];
    v.pose.position.y = pos[1];
    v.pose.position.z = pos[2];
    v.pose.orientation.x = 0.0;
    v.pose.orientation.y = 0.0;
    v.pose.orientation.z = 0.0;
    v.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    v.scale.x = 2.0;
    v.scale.y = 2.0;
    v.scale.z = 2.0;

    // Set the color -- be sure to set alpha to something non-zero!
    v.color.r = 1.0f;
    v.color.g = 0.0f;
    v.color.b = 0.0f;
    v.color.a = 1.0;

    v.lifetime = ros::Duration();
  }

  bool updateNavMesh(recast_ros::RecastPlanner &recast_, const pcl::PolygonMesh &pclMesh, const std::vector<char> &trilabels) // Update NavMesh settings from dynamic rqt_reconfiguration
  {
    nodeHandle_.setParam("cell_size", cellSize_);
    nodeHandle_.setParam("cell_height", cellHeight_);
    nodeHandle_.setParam("agent_height", agentHeight_);
    nodeHandle_.setParam("agent_radius", agentRadius_);
    nodeHandle_.setParam("agent_max_climb", agentMaxClimb_);
    nodeHandle_.setParam("agent_max_slope", agentMaxSlope_);

    std::string temp = "TERRAIN_TYPE", temp1 = "";

    for (size_t i = 1; i < noAreaTypes_; i++)
    {
      temp1 = temp + boost::to_string(i) + "_COST";
      nodeHandle_.setParam(temp1, areaCostList_[i]);
    }

    recast_.stg.cellSize = cellSize_;
    recast_.stg.cellHeight = cellHeight_;
    recast_.stg.agentHeight = agentHeight_;
    recast_.stg.agentRadius = agentRadius_;
    recast_.stg.agentMaxClimb = agentMaxClimb_;
    recast_.stg.agentMaxSlope = agentMaxSlope_;

    ROS_INFO("%f \t %f \t %f\t %f\t %f\t %f", cellSize_, cellHeight_, agentHeight_, agentRadius_, agentMaxClimb_, agentMaxSlope_);

    if (!recast_.build(pclMesh, trilabels))
    {
      ROS_ERROR("Could not build NavMesh");
      return false;
    }
    ROS_INFO("NavMesh is updated");
    return true;
  }
  void buildOriginalMeshVisualization(visualization_msgs::Marker &orgTriList, const pcl::PointCloud<pcl::PointXYZ> &triVerts)
  {
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    c.a = 1.0; // Set Alpha to 1 for visibility

    orgTriList_.colors.resize(triVerts.size());
    orgTriList_.points.resize(triVerts.size());
    for (size_t i = 0; i < triVerts.size(); i++)
    {
      p.x = triVerts.at(i).x;
      p.y = triVerts.at(i).y;
      p.z = triVerts.at(i).z;

      c.r = 0.2;
      c.b = 1;
      c.g = 0.6;

      orgTriList_.colors[i] = c;
      orgTriList_.points[i] = p;
    }
    ROS_INFO("Original Mesh is built\n");
  }

  // Create RViz Markers based on Nav Mesh
  void buildNavMeshVisualization(visualization_msgs::Marker &triList, visualization_msgs::Marker &lineMarkerList,
                                 const pcl::PointCloud<pcl::PointXYZ> &polyVerts,
                                 const std::vector<Eigen::Vector3d> &lineList, const std::vector<unsigned char> areaList)
  {

    ROS_INFO("NavMesh Visualization is built\n");
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    c.a = 1.0; // Set Alpha to 1 for visibility

    lineMarkerList_.colors.resize(lineList.size());
    lineMarkerList_.points.resize(lineList.size());
    for (size_t i = 0; i < lineList.size(); i++)
    {
      p.x = lineList[i][0];
      p.y = lineList[i][1];
      p.z = lineList[i][2];

      c.r = 0;
      c.b = 0;
      c.g = 0;

      lineMarkerList_.colors[i] = c;
      lineMarkerList_.points[i] = p;
    }

    triList_.colors.resize(polyVerts.size());
    triList_.points.resize(polyVerts.size());
    for (int i = 0; i < polyVerts.size(); i++)
    {
      p.x = polyVerts.at(i).x;
      p.y = polyVerts.at(i).y;
      p.z = polyVerts.at(i).z;

      triList_.colors[i] = colourList_[areaList.at(i)];
      triList_.points[i] = p;
    }
  }
  bool addObstacleService(recast_ros::AddObstacleSrv::Request &req, recast_ros::AddObstacleSrv::Response &res)
  {
    visualization_msgs::Marker newMarker;

    float pos[3] = {req.position.x, req.position.y, req.position.z};
    float radius = req.radius;
    float height = req.height;

    obstacleAdded_ = recast_.addRecastObstacle(pos, radius, height);

    if (obstacleAdded_)
    {
      ROS_INFO("Obstacle is added");
      visualizeObstacle(newMarker, pos, (int)obstacleList_.size());

      obstacleList_.push_back(newMarker);
      recast_.update();
    }

    return obstacleAdded_;
  }

  void callbackNavMesh(recast_ros::recast_nodeConfig &config, uint32_t level) // dynamic reconfiguration, update node parameters and class' private variables
  {
    ROS_INFO("Reconfigure Request: %f %f %f %f %f, %f",
             config.cell_size,
             config.cell_height,
             config.agent_height,
             config.agent_radius,
             config.agent_max_climb,
             config.agent_max_slope);

    cellSize_ = config.cell_size;
    cellHeight_ = config.cell_height;
    agentHeight_ = config.agent_height;
    agentRadius_ = config.agent_radius;
    agentMaxClimb_ = config.agent_max_climb;
    agentMaxSlope_ = config.agent_max_slope;

    /*    if (areaCostList_.size() > 0)
      areaCostList_[0] = config.TERRAIN_TYPE0_COST;*/
    if (areaCostList_.size() > 1)
      areaCostList_[1] = config.TERRAIN_TYPE1_COST;
    if (areaCostList_.size() > 2)
      areaCostList_[2] = config.TERRAIN_TYPE2_COST;
    if (areaCostList_.size() > 3)
      areaCostList_[3] = config.TERRAIN_TYPE3_COST;
    if (areaCostList_.size() > 4)
      areaCostList_[4] = config.TERRAIN_TYPE4_COST;
    if (areaCostList_.size() > 5)
      areaCostList_[5] = config.TERRAIN_TYPE5_COST;
    if (areaCostList_.size() > 6)
      areaCostList_[6] = config.TERRAIN_TYPE6_COST;
    if (areaCostList_.size() > 7)
      areaCostList_[7] = config.TERRAIN_TYPE7_COST;
    if (areaCostList_.size() > 8)
      areaCostList_[8] = config.TERRAIN_TYPE8_COST;
    if (areaCostList_.size() > 9)
      areaCostList_[9] = config.TERRAIN_TYPE9_COST;
    if (areaCostList_.size() > 10)
      areaCostList_[10] = config.TERRAIN_TYPE10_COST;
    if (areaCostList_.size() > 11)
      areaCostList_[11] = config.TERRAIN_TYPE11_COST;
    if (areaCostList_.size() > 12)
      areaCostList_[12] = config.TERRAIN_TYPE12_COST;
    if (areaCostList_.size() > 13)
      areaCostList_[13] = config.TERRAIN_TYPE13_COST;
    if (areaCostList_.size() > 14)
      areaCostList_[14] = config.TERRAIN_TYPE14_COST;
    if (areaCostList_.size() > 15)
      areaCostList_[15] = config.TERRAIN_TYPE15_COST;
    if (areaCostList_.size() > 16)
      areaCostList_[16] = config.TERRAIN_TYPE16_COST;
    if (areaCostList_.size() > 17)
      areaCostList_[17] = config.TERRAIN_TYPE17_COST;
    if (areaCostList_.size() > 18)
      areaCostList_[18] = config.TERRAIN_TYPE18_COST;
    if (areaCostList_.size() > 19)
      areaCostList_[19] = config.TERRAIN_TYPE19_COST;

    updateMeshCheck_ = true;
  }

  bool findPathService(recast_ros::RecastPathSrv::Request &req, recast_ros::RecastPathSrv::Response &res)
  {
    ros::WallTime startFunc, endFunc, pathStart, pathEnd;
    startFunc = ros::WallTime::now();
    //Get Input
    ROS_INFO("Input positions are;");
    startX_ = req.startXYZ.x;
    startY_ = req.startXYZ.y;
    startZ_ = req.startXYZ.z;
    ROS_INFO("Start Position x=%f, y=%f, z=%f", startX_, startY_, startZ_);
    goalX_ = req.goalXYZ.x;
    goalY_ = req.goalXYZ.y;
    goalZ_ = req.goalXYZ.z;
    ROS_INFO("Goal Position x=%f, y=%f, z=%f", goalX_, goalY_, goalZ_);
    // test path planning (Detour)
    std::vector<pcl::PointXYZ> path;
    pcl::PointXYZ start;
    start.x = startX_;
    start.y = startY_;
    start.z = startZ_;
    pcl::PointXYZ goal;
    goal.x = goalX_;
    goal.y = goalY_;
    goal.z = goalZ_;

    // query recast/detour for the path
    // TODO: this function should not use pcl as arguments but, std::vector or Eigen...
    pathStart = ros::WallTime::now();
    bool checkStatus = recast_.query(start, goal, path, areaCostList_, noAreaTypes_);
    pathEnd = ros::WallTime::now();

    double exec_time = (pathEnd - pathStart).toNSec() * (1e-6);
    ROS_INFO("Path Query Execution Time (ms): %f", exec_time);

    if (!checkStatus)
    {
      ROS_ERROR("Could not obtain shortest path");
      return checkStatus;
    }
    ROS_INFO("Success: path has size %d", (int)path.size());

    // avoid issues with single-point paths (=goal)
    if (path.size() == 1)
      path.push_back(path[0]);

    // get path
    res.path.resize(path.size());
    for (unsigned int i = 0; i < path.size(); i++)
    {
      // project to navmesh
      unsigned char areaType;
      pcl::PointXYZ pt;
      recast_.getProjection(path[i], pt, areaType);
      path[i] = pt;
      // pass it on to result
      ROS_INFO("path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
      res.path[i].x = path[i].x;
      res.path[i].y = path[i].y;
      res.path[i].z = path[i].z;
    }
    //Last Path Visualization
    setVisualParameters(pathList_, visualization_msgs::Marker::LINE_LIST, "Path Lines", 0);
    setVisualParameters(agentPos_, visualization_msgs::Marker::SPHERE_LIST, "Agent Positions", 1);
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    // Set Alpha to 1 for visibility
    c.a = 1.0;
    c.r = 1.0;
    c.b = 0;
    c.g = 1.0;
    pathList_.colors.clear();
    pathList_.points.clear();
    pathList_.colors.resize(2 * path.size() - 2, c);
    pathList_.points.resize(2 * path.size() - 2);
    agentPos_.points.resize(2);
    agentPos_.colors.resize(2);

    //Add Start Point
    p.x = path[0].x;
    p.y = path[0].y;
    p.z = path[0].z + agentHeight_;

    pathList_.scale.x = 0.1;
    pathList_.points[0] = p;
    c.a = 0.6;
    c.r = 0.0;
    c.g = 1.0;
    c.b = 0.0;
    agentPos_.points[0] = p;
    agentPos_.colors[0] = c;

    int j = 1;
    for (size_t i = 1; i < path.size() - 1; i++)
    {
      p.x = path[i].x;
      p.y = path[i].y;
      p.z = path[i].z + agentHeight_;

      pathList_.points[j] = p;
      pathList_.points[j + 1] = p;
      j = j + 2;
    }
    //Add End Point
    p.x = path[path.size() - 1].x;
    p.y = path[path.size() - 1].y;
    p.z = path[path.size() - 1].z + agentHeight_;
    pathList_.points[pathList_.points.size() - 1] = p;
    c.a = 0.6;
    c.r = 1.0;
    c.g = 0.0;
    c.b = 0.0;
    agentPos_.points[1] = p;
    agentPos_.colors[1] = c;

    RecastPathPub_.publish(pathList_);
    RecastPathPub_.publish(agentPos_);

    endFunc = ros::WallTime::now();

    exec_time = (endFunc - startFunc).toNSec() * (1e-6);

    ROS_INFO("Whole execution time (ms): %f", exec_time);

    return checkStatus;
  }

  bool projectPointService(recast_ros::RecastProjectSrv::Request &req, recast_ros::RecastProjectSrv::Response &res)
  {
    pcl::PointXYZ point;
    point.x = req.point.x;
    point.y = req.point.y;
    point.z = req.point.z;

    pcl::PointXYZ proj;
    unsigned char areaType;
    if (!recast_.getProjection(point, proj, areaType))
    {
      ROS_ERROR("Could not project point");
      return false;
    }

    res.projected_point.x = proj.x;
    res.projected_point.y = proj.y;
    res.projected_point.z = proj.z;
    res.area_type.data = (char)areaType;
    return true;
  }

  void run()
  {
    // load mesh
    pcl::PolygonMesh pclMesh;
    bool loaded_mesh = pcl::io::loadPolygonFileOBJ(path_, pclMesh);
    if (loaded_mesh)
    {
      ROS_INFO("loaded OBJ file (%d polygons)", (int)pclMesh.polygons.size());
    }
    else
    {
      ROS_ERROR("could not load OBJ file");
      return;
    }

    // load triangle labels (a.k.a. area types)
    std::vector<char> trilabels;
    bool loaded_areas = recast_ros::loadAreas(pathAreas_, trilabels);
    if (loaded_areas)
    {
      ROS_INFO("loaded AREAS file (%d polygons)", (int)trilabels.size());
    }
    else
    {
      ROS_WARN("could not load AREAS file... will assume all polygons are of area type 1");
      int n_tris = pclMesh.polygons.size();
      trilabels = std::vector<char>(n_tris, (char)1);
    }

    // build NavMesh
    if (!recast_.build(pclMesh, trilabels))
    {
      ROS_ERROR("Could not build NavMesh");
      return;
    }

    pcl::PolygonMesh::Ptr pclMeshPtr;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp;
    std::vector<Eigen::Vector3d> lineList;
    std::vector<unsigned char> areaList;
    if (!recast_.getNavMesh(pclMeshPtr, temp, lineList, areaList))
      ROS_INFO("Could not retrieve NavMesh");

    // Dynamic Reconfiguration start
    dynamic_reconfigure::Server<recast_ros::recast_nodeConfig> server;
    dynamic_reconfigure::Server<recast_ros::recast_nodeConfig>::CallbackType f;
    f = boost::bind(&RecastNode::callbackNavMesh, this, _1, _2);
    server.setCallback(f);

    // Visualization

    setVisualParameters(triList_, visualization_msgs::Marker::TRIANGLE_LIST, "Nav Mesh Triangles", 2);
    setVisualParameters(orgTriList_, visualization_msgs::Marker::TRIANGLE_LIST, "Original Mesh Triangles", 3);
    setVisualParameters(lineMarkerList_, visualization_msgs::Marker::LINE_LIST, "Nav Mesh Lines", 4);

    pcl::PointCloud<pcl::PointXYZ> polyVerts, triVerts;

    pcl::fromPCLPointCloud2(pclMesh.cloud, triVerts);
    pcl::fromPCLPointCloud2(pclMeshPtr->cloud, polyVerts);

    buildNavMeshVisualization(triList_, lineMarkerList_, polyVerts, lineList, areaList);
    buildOriginalMeshVisualization(orgTriList_, triVerts);

    // infinite loop
    // Index = 0 -> NavMesh, Index = 1 -> Original Mesh, Index = 2 -> Line List
    std::vector<int> listCount = {0, 0, 0, 0};

    while (ros::ok())
    {
      if (updateMeshCheck_ || obstacleAdded_)
      {
        // Clear previous Rviz Markers
        triList_.points.clear();
        triList_.colors.clear();

        lineMarkerList_.points.clear();
        lineMarkerList_.colors.clear();

        polyVerts.clear();
        areaList.clear();
        lineList.clear();

        // Update Navigation mesh based on new config
        if (updateMeshCheck_)
        {
          if (!updateNavMesh(recast_, pclMesh, trilabels))
            ROS_INFO("Map update failed");

          //Clear Obstacles' Markers
          obstacleList_.clear();
          visualization_msgs::Marker deleteMark;
          deleteMark.action = visualization_msgs::Marker::DELETEALL;
          RecastObstaclePub_.publish(deleteMark);
        }

        // Get new navigation mesh
        if (!recast_.getNavMesh(pclMeshPtr, temp, lineList, areaList))
          ROS_INFO("FAILED");

        pcl::fromPCLPointCloud2(pclMeshPtr->cloud, polyVerts);
        // Create Rviz Markers based on new navigation mesh
        buildNavMeshVisualization(triList_, lineMarkerList_, polyVerts, lineList, areaList);

        // clear update requirement flag
        updateMeshCheck_ = false;
        obstacleAdded_ = false;
      }

      // Publish lists
      if (NavMeshPub_.getNumSubscribers() >= 1) // Check Rviz subscribers
      {
        ROS_INFO("Published Navigation Mesh Triangle List No %d", listCount[0]++);
        ROS_INFO("Published Navigation Mesh Line List No %d", listCount[2]++);
        NavMeshPub_.publish(lineMarkerList_);
        NavMeshPub_.publish(triList_);
      }

      if (OriginalMeshPub_.getNumSubscribers() >= 1)
      {
        ROS_INFO("Published Original Mesh Triangle List No %d", listCount[1]++);
        OriginalMeshPub_.publish(orgTriList_);
      }

      if (RecastObstaclePub_.getNumSubscribers() >= 1)
      {
        ROS_INFO("Published obstacles No %d", listCount[3]++);
        for (size_t i = 0; i < obstacleList_.size(); i++)
          RecastObstaclePub_.publish(obstacleList_[i]);
      }

      ros::spinOnce(); // check changes in ros network
      loopRate_.sleep();
    }
  }

  // ros tools
  ros::NodeHandle nodeHandle_;
  ros::Publisher NavMeshPub_;
  ros::Publisher OriginalMeshPub_;
  ros::Publisher RecastPathPub_;
  ros::Publisher RecastObstaclePub_;
  ros::Rate loopRate_;
  ros::ServiceServer servicePlan_;
  ros::ServiceServer serviceAddObstacle_;
  ros::ServiceServer serviceProject_;
  std::string path_;
  std::string pathAreas_;
  recast_ros::RecastPlanner recast_;
  // path planner settings
  double startX_;
  double startY_;
  double startZ_;
  double goalX_;
  double goalY_;
  double goalZ_;
  // recast settings
  float cellSize_;
  float cellHeight_;
  float agentHeight_;
  float agentRadius_;
  float agentMaxClimb_;
  float agentMaxSlope_;
  const int noAreaTypes_; // Number of Area Types
  std::vector<float> areaCostList_;
  bool updateMeshCheck_ = false; // private flag to check whether a map update required or not
  bool obstacleAdded_ = false;   // check whether obstacle is added or not
  //Visualization settings
  visualization_msgs::Marker triList_;
  visualization_msgs::Marker lineMarkerList_;
  visualization_msgs::Marker orgTriList_;
  std::vector<visualization_msgs::Marker> obstacleList_;
  visualization_msgs::Marker pathList_;
  visualization_msgs::Marker agentPos_;
  std::vector<std_msgs::ColorRGBA> colourList_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "recast_node");
  ros::NodeHandle nodeHandle("~");
  RecastNode rcNode(nodeHandle);

  rcNode.run();
  return 0;
}

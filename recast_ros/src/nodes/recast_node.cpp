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
#include "recast_ros/recast_nodeConfig.h"
#include "recast_ros/RecastProjectSrv.h"
#include "recast_ros/RecastPathSrv.h"
#include "recast_ros/AddObstacleSrv.h"
#include <std_srvs/Trigger.h>
#include "recast_ros/InputMeshSrv.h"
#include "recast_ros/RecastPathMsg.h"
#include "recast_ros/RecastGraphNode.h"
#include "recast_ros/RecastGraph.h"
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarker.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <dynamic_reconfigure/server.h>
#include <ros/package.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <fstream>

struct RecastNode
{
  RecastNode(ros::NodeHandle &nodeHandle)
      : nodeHandle_(nodeHandle), noAreaTypes_(20), areaCostList_(noAreaTypes_), colourList_(noAreaTypes_), loopRate_(100.0), graphNodes_(0)
  {
    // ros params
    std::string temp = "TERRAIN_TYPE", temp1 = "";
    nodeHandle_.param("cell_size", cellSize_, 0.1);
    nodeHandle_.param("cell_height", cellHeight_, 0.1);
    nodeHandle_.param("agent_height", agentHeight_, 0.7);
    nodeHandle_.param("agent_radius", agentRadius_, 0.2);
    nodeHandle_.param("agent_max_climb", agentMaxClimb_, 0.41);
    nodeHandle_.param("agent_max_slope", agentMaxSlope_, 60.0);
    nodeHandle_.param("dynamic_reconfigure", dynamicReconfigure_, true);
    nodeHandle_.param("replan_path", replanPath_, true);
    nodeHandle_.param("loop_rate", frequency_, 100.0);
    nodeHandle_.param("rviz_marker_loop_rate", rvizFrequency_, 10);
    nodeHandle_.param("search_buffer_size", searchBufferSize_, 10240);
    nodeHandle_.param("obstacle_radius", obstacleRadius_, 0.5);
    nodeHandle_.param("obstacle_height", obstacleHeight_, 1.0);

    colourCheck_.resize(noAreaTypes_, false);

    // ROS::Rviz marker publishers
    NavMeshPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("navigation_mesh", 1);
    NavMeshFilteredPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("filtered_navigation_mesh", 1);
    NavMeshLinesPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("navigation_mesh_lines", 1);
    NavMeshFilteredLinesPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("filtered_navigation_mesh_lines", 1);
    OriginalMeshPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("original_mesh", 1);
    OriginalMeshLinesPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("original_mesh_lines", 1);
    RecastPathPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("recast_path_lines", 1);
    RecastPathStartPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("recast_path_start", 1);
    RecastPathGoalPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("recast_path_goal", 1);
    RecastObstaclePub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("recast_obstacles", 1);
    graphNodePub_ = nodeHandle_.advertise<visualization_msgs::Marker>("graph_nodes", 1);
    graphConnectionPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("graph_connections", 1);
    graphPub_ = nodeHandle_.advertise<recast_ros::RecastGraph>("graph", 1);

    interactiveMarkerServer_.reset(new interactive_markers::InteractiveMarkerServer("navmesh", "", false));
    interactiveMarkerObstacleServer_.reset(new interactive_markers::InteractiveMarkerServer("obstacles", "", false));

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

    // create service for nodes (server & client)
    servicePlan_ = nodeHandle_.advertiseService("plan_path", &RecastNode::findPathService, this);
    serviceProject_ = nodeHandle_.advertiseService("project_point", &RecastNode::projectPointService, this);
    serviceAddObstacle_ = nodeHandle_.advertiseService("add_obstacle", &RecastNode::addObstacleService, this);
    serviceRemoveAllObstacles_ = nodeHandle_.advertiseService("remove_all_obstacles", &RecastNode::removeAllObstacles, this);
    serviceUpdateMesh_ = nodeHandle_.advertiseService("test_update_parameters", &RecastNode::updateMeshRosParam, this);
    serviceInputMap_ = nodeHandle_.advertiseService("input_mesh", &RecastNode::inputMeshService, this);

    //Create ROS::params for area costs
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

    //publishing ratio
    int publishRate_ = frequency_ / rvizFrequency_;
  }
  int findAvailableColour()
  {
    for (size_t i = 0; i < colourCheck_.size(); i++)
    {
      if (colourCheck_[i])
        continue;
      else
      {
        colourCheck_[i] = true;
        return i;
      }
    }

    ROS_WARN("All the colours are being used, returning index 0");
    return 0;
  }

  bool inputMeshService(recast_ros::InputMeshSrv::Request &req, recast_ros::InputMeshSrv::Response &res)
  {
    newMapReceived_ = true;

    ROS_WARN("New Map is received, building new NavMesh...");

    reference_point_.x = req.reference_point.x;
    reference_point_.y = req.reference_point.y;
    reference_point_.z = req.reference_point.z;

    pcl_conversions::toPCL(req.input_mesh, pclMesh_);

    areaLabels_.resize(req.area_labels.size());

    for (size_t i = 0; i < req.area_labels.size(); i++)
      areaLabels_[i] = req.area_labels[i];

    return newMapReceived_;
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

  //Marker settings
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
      v.scale.x = 0.01;
      v.scale.y = 0.0;
      v.scale.z = 0.0;
    }
    else if ((v.type == visualization_msgs::Marker::SPHERE && nameSpace == "") || v.type == visualization_msgs::Marker::SPHERE_LIST)
    {
      v.scale.x = 0.2;
      v.scale.y = 0.2;
      v.scale.z = 0.2;
    }
    else if (v.type == visualization_msgs::Marker::SPHERE)
    {
      v.scale.x = 0.5;
      v.scale.y = 0.5;
      v.scale.z = 0.5;
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
    v.color.b = 0.0f;
    v.color.a = 0.6f;

    v.lifetime = ros::Duration();
  }
  //Obstacle Marker Settings
  void visualizeObstacle(visualization_msgs::Marker &v, const float *pos, const int &id, const float &radi, const float &height) // Constructs a marker of Triangle List
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
    v.pose.position.z = pos[2] + height / 2;
    v.pose.orientation.x = 0.0;
    v.pose.orientation.y = 0.0;
    v.pose.orientation.z = 0.0;
    v.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    v.scale.x = 2 * radi;
    v.scale.y = 2 * radi;
    v.scale.z = height;

    // Set the color -- be sure to set alpha to something non-zero!
    v.color = colourList_[obstacleColour_];

    v.lifetime = ros::Duration();
  }

  bool updateNavMesh() // Update NavMesh settings from dynamic rqt_reconfiguration or from rosparam set ... command
  {

    recast_.stg.cellSize = cellSize_;
    recast_.stg.cellHeight = cellHeight_;
    recast_.stg.agentHeight = agentHeight_;
    recast_.stg.agentRadius = agentRadius_;
    recast_.stg.agentMaxClimb = agentMaxClimb_;
    recast_.stg.agentMaxSlope = agentMaxSlope_;

    //Builds dtNavMesh, measures performance
    ros::WallTime startFunc, endFunc;
    double exec_time = 0;
    startFunc = ros::WallTime::now();
    if (!recast_.build(pclMesh_, areaLabels_, searchBufferSize_))
    {
      ROS_ERROR("Could not build NavMesh");
      return false;
    }
    endFunc = ros::WallTime::now();
    exec_time = (endFunc - startFunc).toNSec() * (1e-6);
    ROS_INFO("Building NavMesh takes  %f (ms)", exec_time);

    ROS_INFO("NavMesh is updated");
    return true;
  }

  void buildOriginalMeshVisualization(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud)
  {
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    c.a = 1.0; // Set Alpha to 1 for visibility
    c.r = 1.0;
    c.g = 1.0;
    c.b = 1.0;

    //Create pointcloud  for markers of Original input mesh
    pcl::fromPCLPointCloud2(pclMesh_.cloud, *pclCloud);
    int npoly = pclMesh_.polygons.size();

    int id = 0;
    orgTriList_.colors.resize(3 * npoly);
    orgTriList_.points.resize(3 * npoly);

    originalLineList_.colors.resize(6 * npoly, c);
    originalLineList_.points.resize(6 * npoly);

    //Go over all the polygons
    for (int i = 0; i < npoly; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        id = pclMesh_.polygons[i].vertices[j];

        p.x = pclCloud->points[id].x;
        p.y = pclCloud->points[id].y;
        p.z = pclCloud->points[id].z;

        orgTriList_.colors[3 * i + j] = colourList_[areaLabels_[i]];
        colourCheck_[areaLabels_[i]] = true;
        orgTriList_.points[3 * i + j] = p;
      }
      originalLineList_.points[6 * i + 0] = orgTriList_.points[3 * i + 0];
      originalLineList_.points[6 * i + 1] = orgTriList_.points[3 * i + 1];
      originalLineList_.points[6 * i + 2] = orgTriList_.points[3 * i + 1];
      originalLineList_.points[6 * i + 3] = orgTriList_.points[3 * i + 2];
      originalLineList_.points[6 * i + 4] = orgTriList_.points[3 * i + 0];
      originalLineList_.points[6 * i + 5] = orgTriList_.points[3 * i + 2];
    }

    ROS_INFO("Original Mesh is built");
  }

  // Create RViz Markers based on NavMesh
  void buildNavMeshVisualization(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, const std::vector<unsigned char> &trilabels)
  {
    geometry_msgs::Point p;
    std::vector<pcl::PointXYZ> path;
    pcl::PointXYZ centre;
    std::vector<geometry_msgs::Point> triPoints;
    triPoints.reserve(3);
    std_msgs::ColorRGBA c;
    c.a = 1.0; // Set Alpha to 1 for visibility
    c.r = 0;
    c.b = 0;
    c.g = 0;
    centre.x = 0;
    centre.y = 0;
    centre.z = 0;

    pcl::fromPCLPointCloud2(pclNavMesh_->cloud, *pclCloud);
    int npoly = pclNavMesh_->polygons.size();

    int id = 0;

    navMeshLineList_.colors.reserve(6 * npoly);
    navMeshLineList_.points.reserve(6 * npoly);

    navMeshLineListFiltered_.colors.reserve(6 * npoly);
    navMeshLineListFiltered_.points.reserve(6 * npoly);

    navMesh_.colors.reserve(3 * npoly);
    navMesh_.points.reserve(3 * npoly);

    navMeshFiltered_.colors.reserve(3 * npoly);
    navMeshFiltered_.points.reserve(3 * npoly);
    int index = 0;
    double exec_time = 0;

    //Creates dtNavMesh, visualization for RViz
    ros::WallTime startFunc, endFunc;
    for (int i = 0; i < npoly; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        id = pclNavMesh_->polygons[i].vertices[j];

        p.x = pclCloud->points[id].x;
        p.y = pclCloud->points[id].y;
        p.z = pclCloud->points[id].z;

        navMesh_.colors.push_back(colourList_[trilabels[id]]);
        colourCheck_[trilabels[id]] = true;

        navMesh_.points.push_back(p);

        startFunc = ros::WallTime::now();
        centre.x += p.x;
        centre.y += p.y;
        centre.z += p.z;

        if (j == 2)
        {
          centre.x = centre.x / 3.0;
          centre.y = centre.y / 3.0;
          centre.z = centre.z / 3.0;

          p.x = centre.x;
          p.y = centre.y;
          p.z = centre.z;
          c.a = 0.6;

          //Creates NavMeshFiltered markers for RViz, condition is to at least one path from referencePoint to centre of that polygon
          if (recast_.query(reference_point_, centre, path, areaCostList_, noAreaTypes_, noPolygons_))
          {
            navMeshFiltered_.colors.push_back(navMesh_.colors[(3 * +i) + j - 2]);
            navMeshFiltered_.points.push_back(navMesh_.points[(3 * +i) + j - 2]);

            navMeshFiltered_.colors.push_back(navMesh_.colors[(3 * +i) + j - 1]);
            navMeshFiltered_.points.push_back(navMesh_.points[(3 * +i) + j - 1]);

            navMeshFiltered_.colors.push_back(navMesh_.colors[(3 * +i) + j]);
            navMeshFiltered_.points.push_back(navMesh_.points[(3 * +i) + j]);

            navMeshLineListFiltered_.points.push_back(navMesh_.points[(3 * +i) + j - 2]);
            navMeshLineListFiltered_.points.push_back(navMesh_.points[(3 * +i) + j - 1]);
            navMeshLineListFiltered_.points.push_back(navMesh_.points[(3 * +i) + j - 2]);
            navMeshLineListFiltered_.points.push_back(navMesh_.points[(3 * +i) + j]);
            navMeshLineListFiltered_.points.push_back(navMesh_.points[(3 * +i) + j - 1]);
            navMeshLineListFiltered_.points.push_back(navMesh_.points[(3 * +i) + j]);

            index += 3;
          }
          centre.x = 0;
          centre.y = 0;
          centre.z = 0;
        }
        endFunc = ros::WallTime::now();
        exec_time += (endFunc - startFunc).toNSec() * (1e-6);
      }
      navMeshLineList_.points.push_back(navMesh_.points[3 * i + 0]);
      navMeshLineList_.points.push_back(navMesh_.points[3 * i + 1]);
      navMeshLineList_.points.push_back(navMesh_.points[3 * i + 1]);
      navMeshLineList_.points.push_back(navMesh_.points[3 * i + 2]);
      navMeshLineList_.points.push_back(navMesh_.points[3 * i + 0]);
      navMeshLineList_.points.push_back(navMesh_.points[3 * i + 2]);
    }

    //Deletes extra elements
    navMeshFiltered_.points.resize(index);
    //Performance measure
    ROS_INFO("Building Filtered NavMesh takes  %f (ms)", exec_time);
    int numPoly = navMeshFiltered_.points.size() / 3;
    ROS_INFO("NavMesh Visualization is built");
    ROS_INFO("Number of NavMeshFiltered Polygons: %d", numPoly);
  }
  // Create RViz Markers based on NavMesh for NavMesh Graph
  void buildRecastGraphVisualization()
  {
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    c.a = 1; // Set Alpha to 1 for visibility
    c.r = 0.5;
    c.b = 0.5;
    c.g = 0;

    double exec_time = 0;

    int nodeSize = graphNodes_.size();

    graphNodeList_.color.r = c.r;
    graphNodeList_.color.b = c.b;
    graphNodeList_.color.g = c.g;
    graphNodeList_.color.a = 1;

    c.r = 0.390625;
    c.b = 0.13671875;
    c.g = 0.3359375;

    graphConnectionList_.color.r = c.r;
    graphConnectionList_.color.b = c.b;
    graphConnectionList_.color.g = c.g;
    graphConnectionList_.scale.x = 0.05;

    graphNodeList_.points.reserve(nodeSize / 3);
    graphConnectionList_.points.reserve(nodeSize / 3);

    graph_.nodes.clear();
    graph_.nodes.reserve(nodeSize / 3);
    graph_.portals.clear();
    graph_.portals.reserve(nodeSize / 6);

    ros::WallTime startFunc, endFunc;
    startFunc = ros::WallTime::now();
    for (int i = 0; i < nodeSize; i = i + 6)
    {
      // skip nodes that are not accessible
      pcl::PointXYZ pt;
      pt.x = graphNodes_[i];
      pt.y = graphNodes_[i+1];
      pt.z = graphNodes_[i+2];
      std::vector<pcl::PointXYZ> path;
      if (!recast_.query(reference_point_, pt, path, areaCostList_, noAreaTypes_, noPolygons_))
        continue;
      pt.x = graphNodes_[i+3];
      pt.y = graphNodes_[i+4];
      pt.z = graphNodes_[i+5];
      if (!recast_.query(reference_point_, pt, path, areaCostList_, noAreaTypes_, noPolygons_))
        continue;

      // add nodes to visualization list
      p.x = graphNodes_[i];
      p.y = graphNodes_[i + 1];
      p.z = graphNodes_[i + 2] + graphNodeList_.scale.z / 2;

      graphNodeList_.points.push_back(p);
      graphConnectionList_.points.push_back(p);

      p.x = graphNodes_[i + 3];
      p.y = graphNodes_[i + 4];
      p.z = graphNodes_[i + 5] + graphNodeList_.scale.z / 2;

      graphNodeList_.points.push_back(p);
      graphConnectionList_.points.push_back(p);

      // add nodes to our msg structure
      recast_ros::RecastGraphNode node;
      node.point.x = graphNodes_[i];
      node.point.y = graphNodes_[i+1];
      node.point.z = graphNodes_[i+2];
      node.area_type.data = (char)graphNodeAreaTypes_[i/3];
      node.cost.data = areaCostList_[node.area_type.data];
      graph_.nodes.push_back(node);

      node.point.x = graphNodes_[i+3];
      node.point.y = graphNodes_[i+4];
      node.point.z = graphNodes_[i+5];
      node.area_type.data = (char)graphNodeAreaTypes_[i/3+1];
      node.cost.data = areaCostList_[node.area_type.data];
      graph_.nodes.push_back(node);

      // add edge portal
      geometry_msgs::Point portal;
      portal.x = graphNodePortals_[i/2];
      portal.y = graphNodePortals_[i/2+1];
      portal.z = graphNodePortals_[i/2+2];
      graph_.portals.push_back(portal);
    }
    endFunc = ros::WallTime::now();
    exec_time = (endFunc - startFunc).toNSec() * (1e-6);

    //Performance measure
    ROS_INFO("NavMesh Graph Visualization is built");
    ROS_INFO("Building NavMesh Graph takes  %f (ms)", exec_time);
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
      visualizeObstacle(newMarker, pos, (int)obstacleList_.size(), radius, height);

      obstacleList_.push_back(newMarker);
      recast_.update();
    }

    return obstacleAdded_;
  }
  bool removeAllObstacles(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    recast_.clearAllRecastObstacles();
    res.success = true;
    res.message = "Obstacles are removed";
    allObstaclesRemoved_ = true;

    return allObstaclesRemoved_;
  }
  bool updateMeshRosParam(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
  {
    //previous mesh config
    double prev[6] = {cellSize_, cellHeight_, agentHeight_, agentRadius_, agentMaxClimb_, agentMaxSlope_};

    if (nodeHandle_.hasParam("cell_size"))
      nodeHandle_.getParam("cell_size", cellSize_);
    if (nodeHandle_.hasParam("cell_height"))
      nodeHandle_.getParam("cell_height", cellHeight_);
    if (nodeHandle_.hasParam("agent_height"))
      nodeHandle_.getParam("agent_height", agentHeight_);
    if (nodeHandle_.hasParam("agent_radius"))
      nodeHandle_.getParam("agent_radius", agentRadius_);
    if (nodeHandle_.hasParam("agent_max_climb"))
      nodeHandle_.getParam("agent_max_climb", agentMaxClimb_);
    if (nodeHandle_.hasParam("agent_max_slope"))
      nodeHandle_.getParam("agent_max_slope", agentMaxSlope_);
    if (nodeHandle_.hasParam("loop_rate"))
      nodeHandle_.getParam("loop_rate", frequency_);
    if (nodeHandle_.hasParam("rviz_marker_loop_rate"))
      nodeHandle_.getParam("rviz_marker_loop_rate", rvizFrequency_);
    if (nodeHandle_.hasParam("search_buffer_size"))
      nodeHandle_.getParam("search_buffer_size", searchBufferSize_);
    if (nodeHandle_.hasParam("obstacle_radius"))
      nodeHandle_.getParam("obstacle_radius", obstacleRadius_);
    if (nodeHandle_.hasParam("obstacle_height"))
      nodeHandle_.getParam("obstacle_height", obstacleHeight_);

    std::string temp = "TERRAIN_TYPE", temp1 = "";

    //Create ROS::params for area costs
    for (size_t i = 1; i < noAreaTypes_; i++)
    {
      temp1 = temp + boost::to_string(i) + "_COST";
      if (nodeHandle_.hasParam(temp1))
        nodeHandle_.getParam(temp1, areaCostList_[i]);
    }

    publishRate_ = frequency_ / rvizFrequency_;

    res.success = true;
    res.message = "Parameters are being updated";
    // update is required only if mesh parameters are changed
    if (cellSize_ != prev[0] ||
        cellHeight_ != prev[1] ||
        agentHeight_ != prev[2] ||
        agentRadius_ != prev[3] ||
        agentMaxClimb_ != prev[4] ||
        agentMaxSlope_ != prev[5])
    {
      ROS_INFO("Reconfigure Request: %f %f %f %f %f, %f",
               cellSize_,
               cellHeight_,
               agentHeight_,
               agentRadius_,
               agentMaxClimb_,
               agentMaxSlope_);
      updateMeshCheck_ = true;
    }

    return true;
  }
  // dynamic reconfiguration, update node parameters and class' private variables. WARNING: COST & Frequency updates don't require NavMesh Update
  void callbackNavMesh(recast_ros::recast_nodeConfig &config, uint32_t level)
  {

    if (cellSize_ != config.cell_size ||
        cellHeight_ != config.cell_height ||
        agentHeight_ != config.agent_height ||
        agentRadius_ != config.agent_radius ||
        agentMaxClimb_ != config.agent_max_climb ||
        agentMaxSlope_ != config.agent_max_slope)
    {

      ROS_INFO("Reconfigure Request: %f %f %f %f %f, %f",
               config.cell_size,
               config.cell_height,
               config.agent_height,
               config.agent_radius,
               config.agent_max_climb,
               config.agent_max_slope);

      cellSize_ = config.cell_size;
      nodeHandle_.setParam("cell_size", cellSize_);

      cellHeight_ = config.cell_height;
      nodeHandle_.setParam("cell_height", cellHeight_);

      agentHeight_ = config.agent_height;
      nodeHandle_.setParam("agent_height", agentHeight_);

      agentRadius_ = config.agent_radius;
      nodeHandle_.setParam("agent_radius", agentRadius_);

      agentMaxClimb_ = config.agent_max_climb;
      nodeHandle_.setParam("agent_max_climb", agentMaxClimb_);

      agentMaxSlope_ = config.agent_max_slope;
      nodeHandle_.setParam("agent_max_slope", agentMaxSlope_);

      updateMeshCheck_ = true;
    }

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

    std::string temp = "TERRAIN_TYPE", temp1 = "";

    for (size_t i = 1; i < noAreaTypes_; i++)
    {
      temp1 = temp + boost::to_string(i) + "_COST";
      nodeHandle_.setParam(temp1, areaCostList_[i]);
    }

    frequency_ = config.loop_rate;
    nodeHandle_.setParam("loop_rate", frequency_);
    rvizFrequency_ = config.rviz_marker_loop_rate;
    nodeHandle_.setParam("rviz_marker_loop_rate", rvizFrequency_);
    publishRate_ = frequency_ / rvizFrequency_;
    obstacleRadius_ = config.obstacle_radius;
    nodeHandle_.setParam("obstacle_radius", obstacleRadius_);
    obstacleHeight_ = config.obstacle_height;
    nodeHandle_.setParam("obstacle_height", obstacleHeight_);

    loopRate_ = ros::Rate(frequency_);
  }

  bool findPathService(recast_ros::RecastPathSrv::Request &req, recast_ros::RecastPathSrv::Response &res)
  {
    ros::WallTime startFunc, endFunc, pathStart, pathEnd;
    startFunc = ros::WallTime::now();
    //Set visual default visual parameters
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    setVisualParameters(pathList_, visualization_msgs::Marker::LINE_LIST, "Path Lines", 0);
    setVisualParameters(agentStartPos_, visualization_msgs::Marker::SPHERE, "Agent Start Position", 1);
    setVisualParameters(agentGoalPos_, visualization_msgs::Marker::SPHERE, "Agent Goal Position", 10);
    //Get Input
    ROS_INFO("Input positions are;");
    startX_ = req.startXYZ.x;
    startY_ = req.startXYZ.y;
    startZ_ = req.startXYZ.z;
    startSet_ = true;
    ROS_INFO("Start Position x=%f, y=%f, z=%f", startX_, startY_, startZ_);
    goalX_ = req.goalXYZ.x;
    goalY_ = req.goalXYZ.y;
    goalZ_ = req.goalXYZ.z;
    goalSet_ = true;
    ROS_INFO("Goal Position x=%f, y=%f, z=%f", goalX_, goalY_, goalZ_);
    // test path planning (Detour)
    std::vector<pcl::PointXYZ> path;
    pcl::PointXYZ start, goal;
    start.x = startX_;
    start.y = startY_;
    start.z = startZ_;

    goal.x = goalX_;
    goal.y = goalY_;
    goal.z = goalZ_;

    //Add markers for Start & Goal Position
    agentStartPos_.pose.position.x = startX_;
    agentStartPos_.pose.position.y = startY_;
    agentStartPos_.pose.position.z = startZ_ + agentHeight_;
    agentStartPos_.color.a = 0.6;
    agentStartPos_.color.r = 0.0;
    agentStartPos_.color.g = 1.0;
    agentStartPos_.color.b = 0.0;

    agentGoalPos_.pose.position.x = goalX_;
    agentGoalPos_.pose.position.y = goalY_;
    agentGoalPos_.pose.position.z = goalZ_ + agentHeight_;
    agentGoalPos_.color.a = 0.6;
    agentGoalPos_.color.r = 1.0;
    agentGoalPos_.color.g = 0.0;
    agentGoalPos_.color.b = 0.0;

    RecastPathStartPub_.publish(agentStartPos_);
    RecastPathGoalPub_.publish(agentGoalPos_);

    // query recast/detour for the path
    pathStart = ros::WallTime::now();
    bool checkStatus = recast_.query(start, goal, path, areaCostList_, noAreaTypes_, noPolygons_);
    pathEnd = ros::WallTime::now();

    double exec_time = (pathEnd - pathStart).toNSec() * (1e-6);
    ROS_INFO("Path Query Execution Time (ms): %f", exec_time);

    //path fails
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

    c.a = 1.0; // Set Alpha to 1 for visibility
    c.r = 1.0;
    c.b = 0;
    c.g = 1.0;
    pathList_.colors.clear();
    pathList_.points.clear();
    pathList_.colors.resize(2 * path.size() - 2, c);
    pathList_.points.resize(2 * path.size() - 2);

    //Add Start Point
    p.x = path[0].x;
    p.y = path[0].y;
    p.z = path[0].z + agentHeight_;

    pathList_.scale.x = 0.1;
    pathList_.points[0] = p;

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

    //Publish path lines to RViz
    RecastPathPub_.publish(pathList_);

    endFunc = ros::WallTime::now();

    exec_time = (endFunc - startFunc).toNSec() * (1e-6);

    ROS_INFO("Whole path service execution time (ms): %f", exec_time);

    return checkStatus;
  }
  void findPathPlanner()
  {
    ros::WallTime startFunc, endFunc, pathStart, pathEnd;
    startFunc = ros::WallTime::now();
    //Set visual default visual parameters
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    setVisualParameters(pathList_, visualization_msgs::Marker::LINE_LIST, "Path Lines", 0);
    setVisualParameters(agentStartPos_, visualization_msgs::Marker::SPHERE, "Agent Start Position", 1);
    setVisualParameters(agentGoalPos_, visualization_msgs::Marker::SPHERE, "Agent Goal Position", 10);
    //Get Input
    ROS_INFO("Input positions are;");
    ROS_INFO("Start Position x=%f, y=%f, z=%f", startX_, startY_, startZ_);
    ROS_INFO("Goal Position x=%f, y=%f, z=%f", goalX_, goalY_, goalZ_);
    // test path planning (Detour)
    std::vector<pcl::PointXYZ> path;
    pcl::PointXYZ start, goal;
    start.x = startX_;
    start.y = startY_;
    start.z = startZ_;

    goal.x = goalX_;
    goal.y = goalY_;
    goal.z = goalZ_;

    //Add markers for Start & Goal Position
    agentStartPos_.pose.position.x = startX_;
    agentStartPos_.pose.position.y = startY_;
    agentStartPos_.pose.position.z = startZ_ + agentHeight_;
    agentStartPos_.color.a = 0.6;
    agentStartPos_.color.r = 0.0;
    agentStartPos_.color.g = 1.0;
    agentStartPos_.color.b = 0.0;

    agentGoalPos_.pose.position.x = goalX_;
    agentGoalPos_.pose.position.y = goalY_;
    agentGoalPos_.pose.position.z = goalZ_ + agentHeight_;
    agentGoalPos_.color.a = 0.6;
    agentGoalPos_.color.r = 1.0;
    agentGoalPos_.color.g = 0.0;
    agentGoalPos_.color.b = 0.0;

    RecastPathStartPub_.publish(agentStartPos_);
    RecastPathGoalPub_.publish(agentGoalPos_);

    // query recast/detour for the path
    pathStart = ros::WallTime::now();
    bool checkStatus = recast_.query(start, goal, path, areaCostList_, noAreaTypes_, noPolygons_);
    pathEnd = ros::WallTime::now();

    double exec_time = (pathEnd - pathStart).toNSec() * (1e-6);
    ROS_INFO("Path Query Execution Time (ms): %f", exec_time);

    //path fails
    if (!checkStatus)
    {
      ROS_ERROR("Could not obtain shortest path");
      return;
    }
    ROS_INFO("Success: path has size %d", (int)path.size());
    for (unsigned int i = 0; i < path.size(); i++)
    {
      ROS_INFO("Recevied Path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
    }

    // avoid issues with single-point paths (=goal)
    if (path.size() == 1)
      path.push_back(path[0]);

    //Last Path Visualization

    c.a = 1.0; // Set Alpha to 1 for visibility
    c.r = 1.0;
    c.b = 0;
    c.g = 1.0;
    pathList_.colors.clear();
    pathList_.points.clear();
    pathList_.colors.resize(2 * path.size() - 2, c);
    pathList_.points.resize(2 * path.size() - 2);

    //Add Start Point
    p.x = path[0].x;
    p.y = path[0].y;
    p.z = path[0].z + agentHeight_;

    pathList_.scale.x = 0.1;
    pathList_.points[0] = p;

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
    RecastPathPub_.publish(pathList_);

    endFunc = ros::WallTime::now();

    exec_time = (endFunc - startFunc).toNSec() * (1e-6);

    ROS_INFO("Whole path service execution time (ms): %f", exec_time);

    return;
  }

  bool projectPointService(recast_ros::RecastProjectSrv::Request &req, recast_ros::RecastProjectSrv::Response &res)
  {
    pcl::PointXYZ point;
    point.x = req.point.x;
    point.y = req.point.y;
    point.z = req.point.z;

    pcl::PointXYZ proj;
    unsigned char areaType;
    pcl::PointXYZ polyCenter;
    if (!recast_.getProjection(point, proj, areaType, polyCenter))
    {
      ROS_ERROR("Could not project point");
      return false;
    }

    res.projected_point.x = proj.x;
    res.projected_point.y = proj.y;
    res.projected_point.z = proj.z;
    res.area_type.data = (char)areaType;
    res.projected_polygon_center.x = polyCenter.x;
    res.projected_polygon_center.y = polyCenter.y;
    res.projected_polygon_center.z = polyCenter.z;
    return true;
  }
  //Interactive markers for triangles
  void pathPlannerMenu()
  {
    visualization_msgs::InteractiveMarker intMarker;
    visualization_msgs::InteractiveMarkerControl intControl;

    intControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    intControl.always_visible = true;
    intMarker.scale = 0.2;
    intMarker.header.frame_id = "map";

    intMarker.name = navMeshFiltered_.ns + "Interactive";
    intControl.markers.push_back(navMeshFiltered_);
    intMarker.controls.push_back(intControl);
    interactiveMarkerServer_->insert(intMarker);

    menuHandler_.apply(*interactiveMarkerServer_, navMeshFiltered_.ns + "Interactive");
    interactiveMarkerServer_->applyChanges();
  }
  //Interactive markers for obstacles
  void obstaclePlannerMenu()
  {
    visualization_msgs::InteractiveMarker intMarker;
    visualization_msgs::InteractiveMarkerControl intControl;

    for (size_t i = 0; i < obstacleList_.size(); i++)
    {
      intControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      intControl.always_visible = true;
      intMarker.scale = 0.2;
      intMarker.header.frame_id = "map";

      intMarker.name = obstacleList_[i].ns + "Interactive";
      intControl.markers.push_back(obstacleList_[i]);
      intMarker.controls.push_back(intControl);
      interactiveMarkerObstacleServer_->insert(intMarker);

      menuHandlerObs_.apply(*interactiveMarkerObstacleServer_, obstacleList_[i].ns + "Interactive");
    }
    interactiveMarkerObstacleServer_->applyChanges();
  }

  void setStart(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    startX_ = feedback->mouse_point.x;
    startY_ = feedback->mouse_point.y;
    startZ_ = feedback->mouse_point.z;

    startSet_ = true;

    if (goalSet_)
    {
      findPathPlanner();
    }
  }
  void setGoal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    goalX_ = feedback->mouse_point.x;
    goalY_ = feedback->mouse_point.y;
    goalZ_ = feedback->mouse_point.z;

    goalSet_ = true;

    if (startSet_)
    {
      findPathPlanner();
    }
  }
  //Menu option to delete all obstacles
  void removeAllObstaclesMenu(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    recast_.clearAllRecastObstacles();
    allObstaclesRemoved_ = true;
  }
  //Menu option to delete specific obstacle
  void deleteObstaclePlanner(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    float hitPos[3], actualPos[3];
    hitPos[0] = feedback->mouse_point.x;
    hitPos[1] = feedback->mouse_point.y;
    hitPos[2] = feedback->mouse_point.z;

    std::vector<visualization_msgs::Marker>::iterator temp = obstacleList_.begin();

    for (unsigned int i = 0; i < obstacleList_.size() && !obstacleRemoved_; ++i)
    {
      //Tolerance values for check obstacle position within range of clicking
      float tolX = obstacleList_[i].scale.x / 2.0f + 0.1f;
      float tolY = obstacleList_[i].scale.y / 2.0f + 0.1f;
      float tolZ = obstacleList_[i].scale.z;

      if (fabs(obstacleList_[i].pose.position.x - hitPos[0]) <= tolX &&
          fabs(obstacleList_[i].pose.position.y - hitPos[1]) <= tolY &&
          fabs(obstacleList_[i].pose.position.z - hitPos[2]) <= tolZ)
      {
        //Gets actualPosition of obstacle marker (centre of cylinder)
        actualPos[0] = obstacleList_[i].pose.position.x;
        actualPos[1] = obstacleList_[i].pose.position.y;
        actualPos[2] = obstacleList_[i].pose.position.z - obstacleList_[i].scale.z / 2; // see visualizeObstacle for the reason of substraction
        obstacleList_[i].action = visualization_msgs::Marker::DELETE;

        obstacleList_.erase(temp);
        obstacleRemoved_ = true;
      }
      temp++;
    }
    visualization_msgs::MarkerArray ma;
    ma.markers = obstacleList_;
    RecastObstaclePub_.publish(ma);
    recast_.removeRecastObstacle(actualPos);
    recast_.update();

    ROS_WARN("Obstacle at %f %f %f is deleted", actualPos[0], actualPos[1], actualPos[2]);
  }
  //Menu option to add obstacles
  void addObstaclePlanner(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    float hitPos[3];
    hitPos[0] = feedback->mouse_point.x;
    hitPos[1] = feedback->mouse_point.y;
    hitPos[2] = feedback->mouse_point.z;

    visualization_msgs::Marker newMarker;

    obstacleAdded_ = recast_.addRecastObstacle(hitPos, obstacleRadius_, obstacleHeight_);

    if (obstacleAdded_)
    {
      ROS_WARN("Obstacle at %f %f %f is added", hitPos[0], hitPos[1], hitPos[2]);
      visualizeObstacle(newMarker, hitPos, (int)obstacleList_.size(), obstacleRadius_, obstacleHeight_);

      obstacleList_.push_back(newMarker);
      recast_.update();
    }
    else
    {
      ROS_ERROR("Obstacle adding is failed");
    }
  }

  //Drop down menu for interactive markers
  //Triangles are handled by menuHandler_, obstacles are handled by menuHandlerObs_
  void initMenu()
  {
    setStartPos_ = menuHandler_.insert("Set Start Position", boost::bind(&RecastNode::setStart, this, _1));
    setGoalPos_ = menuHandler_.insert("Set Goal Position", boost::bind(&RecastNode::setGoal, this, _1));
    addObstacle_ = menuHandler_.insert("Add Obstacle", boost::bind(&RecastNode::addObstaclePlanner, this, _1));
    removeAllObstacles_ = menuHandler_.insert("Remove All Obstacles", boost::bind(&RecastNode::removeAllObstaclesMenu, this, _1));
    deleteObstacle_ = menuHandlerObs_.insert("Delete Obstacle", boost::bind(&RecastNode::deleteObstaclePlanner, this, _1));
  }

  void run()
  {
    //Waits for input map
    while (!newMapReceived_)
    {
      ROS_WARN_ONCE("Waiting for map point cloud");
      ros::spinOnce(); // check changes in ros network
      loopRate_.sleep();
    }

    //Create variables for conversion dtNavMesh to pcl::PolygonMesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclNavMeshCloud, pclOriginalCloud;
    pclOriginalCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<Eigen::Vector3d> lineList;
    std::vector<unsigned char> areaList;

    if (dynamicReconfigure_)
    {
      dynamicCallback_ = boost::bind(&RecastNode::callbackNavMesh, this, _1, _2);
      dynamicReconfigureServer_.setCallback(dynamicCallback_);
    }

    // Visualization

    initMenu();
    setVisualParameters(navMesh_, visualization_msgs::Marker::TRIANGLE_LIST, "NavMesh Triangles", 2);
    setVisualParameters(navMeshFiltered_, visualization_msgs::Marker::TRIANGLE_LIST, "Filtered NavMesh Triangles", 12);
    setVisualParameters(orgTriList_, visualization_msgs::Marker::TRIANGLE_LIST, "Original Mesh Triangles", 3);
    setVisualParameters(navMeshLineList_, visualization_msgs::Marker::LINE_LIST, "NavMesh Lines", 4);
    setVisualParameters(navMeshLineListFiltered_, visualization_msgs::Marker::LINE_LIST, "Filtered NavMesh  Lines", 13);
    setVisualParameters(originalLineList_, visualization_msgs::Marker::LINE_LIST, "Original Mesh Lines", 8);
    setVisualParameters(graphNodeList_, visualization_msgs::Marker::SPHERE_LIST, "Graph Nodes", 15);
    setVisualParameters(graphConnectionList_, visualization_msgs::Marker::LINE_LIST, "Graph Connections", 16);

    buildOriginalMeshVisualization(pclOriginalCloud);
    //Assigns first available colour to obstacles
    obstacleColour_ = findAvailableColour();

    /*     //Builds initial Mesh Visualization
    buildNavMeshVisualization(pclNavMeshCloud, areaList);
    buildRecastGraphVisualization();
 */
    // infinite loop
    // Index = 0 -> NavMesh, Index = 1 -> Original Mesh, Index = 2 -> NavMesh Line List, Index = 3 -> Obstacle List, Index = 4 -> Original Mesh Line List
    // Index = 5 -> Filtered NavMesh Triangle List, Index = 6 -> Filtered NavMesh Line List, Index = 7 -> Graph Nodes List, Index = 8 -> Graph Connection List
    std::vector<int> listCount(10, 0);
    // counter for the ratio between RViz Marker publishing rate and ROS::services cycle frequency
    int loopCount = 0;

    //Main loop
    while (ros::ok())
    {

      if (updateMeshCheck_ || obstacleAdded_ || obstacleRemoved_ || newMapReceived_ || allObstaclesRemoved_)
      {
        // Clear previous Rviz Markers
        navMesh_.points.clear();
        navMesh_.colors.clear();

        navMeshFiltered_.points.clear();
        navMeshFiltered_.colors.clear();

        navMeshLineList_.points.clear();
        navMeshLineList_.colors.clear();

        navMeshLineListFiltered_.points.clear();
        navMeshLineListFiltered_.colors.clear();

        //Clears previous map data
        graphNodes_.clear();
        graphNodePortals_.clear();
        graphNodeAreaTypes_.clear();
        areaList.clear();
        lineList.clear();

        // Update Navigation mesh based on new config
        if (updateMeshCheck_ || newMapReceived_)
        {
          if (!updateNavMesh())
            ROS_ERROR("Map update failed");

          //Clear Obstacles' Markers
          visualization_msgs::MarkerArray deleteMarkers;
          deleteMarkers.markers.resize(1);
          deleteMarkers.markers[0].action = visualization_msgs::Marker::DELETEALL;
          visualization_msgs::Marker deleteMark;
          deleteMark.action = visualization_msgs::Marker::DELETEALL;  
          RecastObstaclePub_.publish(deleteMarkers);
          obstacleList_.clear();  
          graphConnectionPub_.publish(deleteMark);
          graphNodePub_.publish(deleteMark);
          colourCheck_.resize(noAreaTypes_, false);
        }
        if (allObstaclesRemoved_)
        {
          //Clear Obstacles' Markers
          visualization_msgs::MarkerArray deleteMarkers;
          deleteMarkers.markers.resize(1);
          deleteMarkers.markers[0].action = visualization_msgs::Marker::DELETEALL;
          RecastObstaclePub_.publish(deleteMarkers);  
          obstacleList_.clear();
        }

        // Get new navigation mesh
        if (!recast_.getNavMesh(pclNavMesh_, pclNavMeshCloud, lineList, areaList, noPolygons_))
          ROS_ERROR("GetNavMesh Failed");
        else
          ROS_INFO("Number of NavMesh Polygons: %d", noPolygons_);

        if (recast_.drawRecastGraph(graphNodes_, graphNodePortals_, graphNodeAreaTypes_))
        {
          graphNodeList_.points.clear();
          graphNodeList_.colors.clear();

          graphConnectionList_.points.clear();
          graphConnectionList_.colors.clear();
        }
        else
          ROS_ERROR("Recast Graph is failed");

        // Create Rviz Markers based on new navigation mesh
        buildNavMeshVisualization(pclNavMeshCloud, areaList);
        buildRecastGraphVisualization();
        //updates interactive markers' menu
        interactiveMarkerServer_.reset(new interactive_markers::InteractiveMarkerServer("navmesh", "", false));
        pathPlannerMenu();
        interactiveMarkerObstacleServer_.reset(new interactive_markers::InteractiveMarkerServer("obstacles", "", false));
        if (obstacleAdded_ || obstacleRemoved_ || allObstaclesRemoved_)
          obstaclePlannerMenu();

        // clear update requirement flag
        updateMeshCheck_ = false;
        obstacleAdded_ = false;
        obstacleRemoved_ = false;
        newMapReceived_ = false;
        allObstaclesRemoved_ = false;
        loopCount = 0;
      }

      // re-plan last path
      if (replanPath_ && startSet_ && goalSet_)
      {
        findPathPlanner();
      }

      // publish things in this cycle?
      if (loopCount % publishRate_ == 0)
      {
        // Publish lists
        if (NavMeshPub_.getNumSubscribers() >= 1) // Check Rviz subscribers
        {
          ROS_INFO("Published Navigation Mesh Triangle List No %d", listCount[0]++);
          NavMeshPub_.publish(navMesh_);
        }
        if (OriginalMeshPub_.getNumSubscribers() >= 1)
        {
          ROS_INFO("Published Original Mesh Triangle List No %d", listCount[1]++);
          OriginalMeshPub_.publish(orgTriList_);
        }
        if (NavMeshLinesPub_.getNumSubscribers() >= 1) // Check Rviz subscribers
        {
          ROS_INFO("Published Navigation Mesh Line List No %d", listCount[2]++);
          NavMeshLinesPub_.publish(navMeshLineList_);
        }
        if (RecastObstaclePub_.getNumSubscribers() >= 1)
        {
          ROS_INFO("Published obstacles No %d", listCount[3]++);
          visualization_msgs::MarkerArray ma;
          ma.markers = obstacleList_;
          RecastObstaclePub_.publish(ma);
        }
        if (OriginalMeshLinesPub_.getNumSubscribers() >= 1)
        {
          ROS_INFO("Published Original Mesh Line List No %d", listCount[4]++);
          OriginalMeshLinesPub_.publish(originalLineList_);
        }
        if (NavMeshFilteredPub_.getNumSubscribers() >= 1) // Check Rviz subscribers
        {
          ROS_INFO("Published Filtered Navigation Mesh Triangle List No %d", listCount[5]++);
          NavMeshFilteredPub_.publish(navMeshFiltered_);
        }
        if (NavMeshFilteredLinesPub_.getNumSubscribers() >= 1) // Check Rviz subscribers
        {
          ROS_INFO("Published Navigation Filtered Mesh Line List No %d", listCount[6]++);
          NavMeshFilteredLinesPub_.publish(navMeshLineListFiltered_);
        }
        if (graphNodePub_.getNumSubscribers() >= 1)
        {
          ROS_INFO("Published Graph Nodes No %d", listCount[7]++);
          graphNodePub_.publish(graphNodeList_);
        }
        if (graphConnectionPub_.getNumSubscribers() >= 1)
        {
          ROS_INFO("Published Graph Connection No %d", listCount[8]++);
          graphConnectionPub_.publish(graphConnectionList_);
        }
        if (graphPub_.getNumSubscribers() >= 1)
        {
          ROS_INFO("Published Graph No %d", listCount[9]++);
          graphPub_.publish(graph_);
        }
        loopCount = 0;
      }

      ros::spinOnce(); // check changes in ros network
      loopRate_.sleep();
      loopCount++;
    }
  }

  // ros tools
  ros::NodeHandle nodeHandle_;
  ros::Publisher NavMeshPub_;
  ros::Publisher NavMeshFilteredPub_;
  ros::Publisher NavMeshLinesPub_;
  ros::Publisher NavMeshFilteredLinesPub_;
  ros::Publisher OriginalMeshPub_;
  ros::Publisher OriginalMeshLinesPub_;
  ros::Publisher RecastPathPub_;
  ros::Publisher RecastObstaclePub_;
  ros::Publisher RecastPathStartPub_;
  ros::Publisher RecastPathGoalPub_;
  ros::Publisher graphNodePub_;
  ros::Publisher graphConnectionPub_;
  ros::Publisher graphPub_;
  ros::Rate loopRate_;
  double frequency_ = 100.0;
  int rvizFrequency_ = 10;
  // Dynamic Reconfiguration start
  dynamic_reconfigure::Server<recast_ros::recast_nodeConfig> dynamicReconfigureServer_;
  dynamic_reconfigure::Server<recast_ros::recast_nodeConfig>::CallbackType dynamicCallback_;
  // ratio between frequency and rvizFrequency
  int publishRate_ = 10;
  ros::ServiceServer servicePlan_;
  ros::ServiceServer serviceAddObstacle_;
  ros::ServiceServer serviceRemoveAllObstacles_;
  ros::ServiceServer serviceUpdateMesh_;
  ros::ServiceServer serviceProject_;
  ros::ServiceServer serviceInputMap_;
  recast_ros::RecastPlanner recast_;
  pcl::PolygonMesh pclMesh_;
  pcl::PolygonMesh::Ptr pclNavMesh_;
  // path planner settings
  pcl::PointXYZ reference_point_;
  double startX_;
  double startY_;
  double startZ_;
  double goalX_;
  double goalY_;
  double goalZ_;
  int noPolygons_ = 0;
  // recast settings
  double cellSize_;
  double cellHeight_;
  double agentHeight_;
  double agentRadius_;
  double agentMaxClimb_;
  double agentMaxSlope_;
  //Default obstacle parameters
  double obstacleRadius_;
  double obstacleHeight_;
  int obstacleColour_;
  // Number of Area Types
  const int noAreaTypes_;
  int searchBufferSize_; // Search nodePool buffer size
  bool dynamicReconfigure_;
  bool replanPath_;
  std::vector<float> areaCostList_;
  std::vector<char> areaLabels_;
  std::vector<float> graphNodes_;
  std::vector<float> graphNodePortals_;
  std::vector<unsigned char> graphNodeAreaTypes_;
  bool updateMeshCheck_ = false; // private flag to check whether a map update required or not
  bool obstacleAdded_ = false;   // check whether obstacle is added or not
  bool obstacleRemoved_ = false;
  bool allObstaclesRemoved_ = false;
  bool newMapReceived_ = false;
  bool startSet_ = false;
  bool goalSet_ = false;
  //Visualization settings
  interactive_markers::MenuHandler::EntryHandle setStartPos_;
  interactive_markers::MenuHandler::EntryHandle setGoalPos_;
  interactive_markers::MenuHandler::EntryHandle deleteObstacle_;
  interactive_markers::MenuHandler::EntryHandle addObstacle_;
  interactive_markers::MenuHandler::EntryHandle removeAllObstacles_;

  interactive_markers::MenuHandler menuHandler_;
  interactive_markers::MenuHandler menuHandlerObs_;
  std::vector<visualization_msgs::InteractiveMarker> intMarkerVec_;
  std::vector<visualization_msgs::InteractiveMarkerControl> intControlVec_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerObstacleServer_;

  visualization_msgs::Marker navMesh_;
  visualization_msgs::Marker navMeshFiltered_;
  visualization_msgs::Marker navMeshLineList_;
  visualization_msgs::Marker navMeshLineListFiltered_;
  visualization_msgs::Marker orgTriList_;
  visualization_msgs::Marker originalLineList_;
  visualization_msgs::Marker graphNodeList_;
  visualization_msgs::Marker graphConnectionList_;
  recast_ros::RecastGraph graph_;
  std::vector<visualization_msgs::Marker> obstacleList_;
  visualization_msgs::Marker pathList_;
  visualization_msgs::Marker agentStartPos_;
  visualization_msgs::Marker agentGoalPos_;
  std::vector<std_msgs::ColorRGBA> colourList_;
  //Stores unused colours wrt to indexes
  std::vector<bool> colourCheck_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "recast_node");
  ros::NodeHandle nodeHandle("~");
  RecastNode rcNode(nodeHandle);

  rcNode.run();
  return 0;
}

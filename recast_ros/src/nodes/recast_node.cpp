#include "recast_ros/RecastPlanner.h"
#include "recast_ros/recast_nodeConfig.h"
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include "recast_ros/recast_path_coords.h"
#include "recast_ros/recast_path_planning.h"
#include <ros/package.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>
#include <fstream>

struct RecastNode
{
  RecastNode(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle), areaCostList_(noAreaTypes_), loopRate_(1)
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
    NavMeshPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("NavMeshPublish", 1);
    OriginalMeshPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("OriginalMeshPub", 1);

    // create service (server & client)
    service_ = nodeHandle_.advertiseService("recast_path_planning", &RecastNode::findPathService, this);

    for (size_t i = 0; i < noAreaTypes_; i++)
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

  void setVisualParametersTriList(visualization_msgs::Marker &v) // Constructs a marker of Triangle List
  {
    v.header.frame_id = "map";
    v.type = visualization_msgs::Marker::TRIANGLE_LIST;
    v.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    v.ns = "Triangle";
    v.id = 0;

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
    v.scale.x = 1.0;
    v.scale.y = 1.0;
    v.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    v.color.r = 0.0f;
    v.color.g = 1.0f;
    v.color.b = 0.0f;
    v.color.a = 1.0;

    v.lifetime = ros::Duration();
  }
  void setVisualParametersLineList(visualization_msgs::Marker &v) // Constructs a marker of Line List
  {
    v.header.frame_id = "map";
    v.type = visualization_msgs::Marker::LINE_LIST;
    v.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    v.ns = "Lines";
    v.id = 1;

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
    v.scale.x = 0.03;
    v.scale.y = 0.0;
    v.scale.z = 0.0;

    // Set the color -- be sure to set alpha to something non-zero!
    v.color.r = 0.0f;
    v.color.g = 1.0f;
    v.color.b = 0.0f;
    v.color.a = 1.0;

    v.lifetime = ros::Duration();
  }
  bool updateNavMesh(recastapp::RecastPlanner &recast_, const pcl::PolygonMesh &pclMesh, const std::vector<char> &trilabels) // Update NavMesh settings from dynamic rqt_reconfiguration
  {
    nodeHandle_.setParam("cell_size", cellSize_);
    nodeHandle_.setParam("cell_height", cellHeight_);
    nodeHandle_.setParam("agent_height", agentHeight_);
    nodeHandle_.setParam("agent_radius", agentRadius_);
    nodeHandle_.setParam("agent_max_climb", agentMaxClimb_);
    nodeHandle_.setParam("agent_max_slope", agentMaxSlope_);

    std::string temp = "TERRAIN_TYPE", temp1 = "";

    for (size_t i = 0; i < noAreaTypes_; i++)
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

  // Create RViz Markers based on Nav Mesh
  void buildNavMeshVisualization(visualization_msgs::Marker &triList, visualization_msgs::Marker &lineMarkerList,
                                 visualization_msgs::Marker &orgTriList, const pcl::PointCloud<pcl::PointXYZ> &polyVerts,
                                 const pcl::PointCloud<pcl::PointXYZ> &triVerts, const std::vector<Eigen::Vector3d> &lineList, const std::vector<unsigned char> areaList)
  {

    ROS_INFO("NavMesh Visualization is built\n");
    geometry_msgs::Point p;
    std_msgs::ColorRGBA c;
    c.a = 1.0; // Set Alpha to 1 for visibility

    for (size_t i = 0; i < triVerts.size(); i++)
    {

      p.x = triVerts.at(i).x;
      p.y = triVerts.at(i).y;
      p.z = triVerts.at(i).z;

      c.r = 0.2;
      c.b = 1;
      c.g = 0.6;

      orgTriList.colors.push_back(c);
      orgTriList.points.push_back(p);
    }

    for (size_t i = 0; i < lineList.size(); i++)
    {
      p.x = lineList[i][0];
      p.y = lineList[i][1];
      p.z = lineList[i][2];

      c.r = 0;
      c.b = 0;
      c.g = 0;

      lineMarkerList.colors.push_back(c);
      lineMarkerList.points.push_back(p);
    }

    for (int i = 0; i < polyVerts.size(); i++)
    {
      p.x = polyVerts.at(i).x;
      p.y = polyVerts.at(i).y;
      p.z = polyVerts.at(i).z;

      if (areaList.at(i) == 0x01)
      {
        c.r = 0.75294117647;
        c.b = 0.75294117647;
        c.g = 0.75294117647;
      }
      else if (areaList.at(i) == 0x02)
      {
        c.r = 0.8;
        c.b = 0.0;
        c.g = 0.0;
      }

      triList.colors.push_back(c);
      triList.points.push_back(p);
    }
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

    if (areaCostList_.size() > 0)
      areaCostList_[0] = config.TERRAIN_TYPE0_COST;
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
    if (areaCostList_.size() > 20)
      areaCostList_[20] = config.TERRAIN_TYPE20_COST;

    updateMeshCheck_ = true;
  }
  bool findPathService(recast_ros::recast_path_planning::Request &req, recast_ros::recast_path_planning::Response &res)
  {
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

    // TODO: this function should not use pcl as arguments but, std::vector or Eigen...
    bool checkStatus = recast_.query(start, goal, path, areaCostList_, noAreaTypes_);

    if (!checkStatus)
    {
      ROS_ERROR("Could not obtain shortest path");
      return checkStatus;
    }
    ROS_INFO("success");

    res.pathSrv.resize(path.size());
    for (unsigned int i = 0; i < path.size(); i++)
    {
      ROS_INFO("path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
      res.pathSrv[i].x = path[i].x;
      res.pathSrv[i].y = path[i].y;
      res.pathSrv[i].z = path[i].z;
    }

    return true;
  }

  bool findPathManual()
  {
    //Get Input
    ROS_INFO("Input positions are;");

    ROS_INFO("Start Position x=%f, y=%f, z=%f", startX_, startY_, startZ_);
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

    // TODO: this function should not use pcl as arguments but, std::vector or Eigen...
    bool checkStatus = recast_.query(start, goal, path, areaCostList_, noAreaTypes_);

    if (!checkStatus)
    {
      ROS_ERROR("Could not obtain shortest path");
      return checkStatus;
    }
    ROS_INFO("success");

    for (unsigned int i = 0; i < path.size(); i++)
    {
      ROS_INFO("path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
    }

    return checkStatus;
  }

  void run()
  {
    // load
    pcl::PolygonMesh pclMesh;
    std::vector<char> trilabels;
    pcl::io::loadPolygonFileOBJ(path_, pclMesh);
    ROS_INFO("loaded OBJ file");
    loadAreas(pathAreas_, trilabels);
    ROS_INFO("loaded AREAS file");

    // build NavMesh
    if (!recast_.build(pclMesh, trilabels))
    {
      ROS_ERROR("Could not build NavMesh");
      return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp;
    std::vector<Eigen::Vector3d> lineList;
    pcl::PolygonMesh::Ptr pclMeshPtr;
    std::vector<unsigned char> areaList;
    if (!recast_.getNavMesh(pclMeshPtr, temp, lineList, areaList))
      ROS_INFO("FAILED");

    // Dynamic Reconfiguration start
    dynamic_reconfigure::Server<recast_ros::recast_nodeConfig> server;
    dynamic_reconfigure::Server<recast_ros::recast_nodeConfig>::CallbackType f;
    f = boost::bind(&RecastNode::callbackNavMesh, this, _1, _2);
    server.setCallback(f);

    // Visualization
    visualization_msgs::Marker triList;
    visualization_msgs::Marker lineMarkerList;
    visualization_msgs::Marker orgTriList;

    setVisualParametersTriList(triList);
    setVisualParametersTriList(orgTriList);
    setVisualParametersLineList(lineMarkerList);
    pcl::PointCloud<pcl::PointXYZ> polyVerts, triVerts;

    pcl::fromPCLPointCloud2(pclMesh.cloud, triVerts);
    pcl::fromPCLPointCloud2(pclMeshPtr->cloud, polyVerts);

    buildNavMeshVisualization(triList, lineMarkerList, orgTriList, polyVerts, triVerts, lineList, areaList);

    // infinite loop
    std::vector<int> listCount = {0, 0, 0}; // Index = 0 -> NavMesh, Index = 1 -> Original Mesh, Index = 2 -> Line List
    while (ros::ok())
    {
      // Publish lists
      if (NavMeshPub_.getNumSubscribers() >= 1) // Check Rviz subscribers
      {
        ROS_INFO("Published Navigation Mesh Triangle List No %d", listCount[0]++);
        ROS_INFO("Published Navigation Mesh Line List No %d", listCount[2]++);
        NavMeshPub_.publish(lineMarkerList);
        NavMeshPub_.publish(triList);
      }

      if (OriginalMeshPub_.getNumSubscribers() >= 1)
      {
        ROS_INFO("Published Original Mesh Triangle List No %d", listCount[1]);
        OriginalMeshPub_.publish(orgTriList);
        listCount[1]++;
      }

      if (updateMeshCheck_)
      {
        // Clear previous Rviz Markers
        triList.points.clear();
        triList.colors.clear();

        lineMarkerList.points.clear();
        lineMarkerList.colors.clear();

        orgTriList.points.clear();
        orgTriList.colors.clear();

        polyVerts.clear();
        areaList.clear();
        lineList.clear();
        // Update Navigation mesh based on new config
        if (!updateNavMesh(recast_, pclMesh, trilabels))
          ROS_INFO("Map update failed");
        // Get new navigation mesh
        if (!recast_.getNavMesh(pclMeshPtr, temp, lineList, areaList))
          ROS_INFO("FAILED");

        pcl::fromPCLPointCloud2(pclMeshPtr->cloud, polyVerts);
        // Create Rviz Markers based on new navigation mesh
        buildNavMeshVisualization(triList, lineMarkerList, orgTriList, polyVerts, triVerts, lineList, areaList);
        updateMeshCheck_ = false; // clear update requirement flag
      }
      ros::spinOnce(); // check changes in ros network
      loopRate_.sleep();
    }
  }

  // ros tools
  ros::NodeHandle nodeHandle_;
  ros::Publisher NavMeshPub_;
  ros::Publisher OriginalMeshPub_;
  ros::Rate loopRate_;
  ros::ServiceServer service_;
  std::string path_;
  std::string pathAreas_;
  recastapp::RecastPlanner recast_;
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
  const int noAreaTypes_ = 21;      // Number of Area Types
  std::vector<float> areaCostList_; 
  bool updateMeshCheck_ = false;    // private flag to check whether a map update required or not
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "recast_node");
  ros::NodeHandle nodeHandle("~");
  RecastNode rcNode(nodeHandle);

  rcNode.run();
  return 0;
}

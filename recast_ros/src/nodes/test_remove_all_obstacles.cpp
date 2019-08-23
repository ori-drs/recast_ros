#include "ros/ros.h"
#include "recast_ros/RemoveAllObstaclesSrv.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_remove_all_obstacles");

    ros::NodeHandle clientNode("~");
    ros::ServiceClient client = clientNode.serviceClient<recast_ros::RemoveAllObstaclesSrv>("/recast_node/remove_all_obstacles");

    recast_ros::RemoveAllObstaclesSrv srv;


    if (client.call(srv))
    {
        ROS_INFO("Obstacles are removed, mesh is updated");
    }
    else
    {
        ROS_INFO("Failed to remove obstacle(s)");
        return 1;
    }

    return 0;
}

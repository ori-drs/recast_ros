#include "ros/ros.h"
#include <std_srvs/Trigger.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_remove_all_obstacles");

    ros::NodeHandle clientNode("~");
    ros::ServiceClient client = clientNode.serviceClient<std_srvs::Trigger>("/recast_node/remove_all_obstacles");

    std_srvs::Trigger srv;


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

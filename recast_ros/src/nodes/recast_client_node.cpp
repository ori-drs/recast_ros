#include "ros/ros.h"
#include "recast_ros/RecastPathSrv.h"
#include <pcl/common/io.h>
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recast_client_node");

    ros::NodeHandle clientNode("~");
    ros::ServiceClient client = clientNode.serviceClient<recast_ros::RecastPathSrv>("RecastPathSrv");

    recast_ros::RecastPathSrv srv;
    geometry_msgs::Point firstArr, secondArr;

/*    for (char **ptr = argv++; ptr; ptr++)
    {
        std::cout << *ptr << std::endl;
    }*/

    firstArr.x = 13.1;
    firstArr.y = 5.18;
    firstArr.z = -1.16;

    secondArr.x = 8.29;
    secondArr.y = 2.14;
    secondArr.z = -1.1;
    
    srv.request.startXYZ = firstArr;
    srv.request.goalXYZ = secondArr;
    while (true)
    {
        if (client.call(srv))
        {
            std::vector<geometry_msgs::Point> path = srv.response.pathSrv;
            for (unsigned int i = 0; i < path.size(); i++)
            {
                ROS_INFO("recevied path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
            }
        }
        else
        {
            ROS_INFO("Failed to receive Path");
            return 1;
        }
    }
    return 0;
}

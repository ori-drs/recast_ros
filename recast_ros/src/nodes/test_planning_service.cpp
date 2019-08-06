#include "ros/ros.h"
#include "recast_ros/recast_path_planning.h"
#include <pcl/common/io.h>
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_planning_service");

    ros::NodeHandle clientNode("~");
    ros::ServiceClient client = clientNode.serviceClient<recast_ros::recast_path_planning>("/recast_node/recast_path_planning");

    geometry_msgs::Point firstArr, secondArr;
    std::vector<double> myCords(6,1);

   for (int i = 1; i < argc; i++)
    {
        myCords[i-1] = atof(argv[i]);
        std::cout << myCords[i-1] << std::endl;
    }

    firstArr.x = myCords[0];
    firstArr.y = myCords[1];
    firstArr.z = myCords[2];

    secondArr.x = myCords[3];
    secondArr.y = myCords[4];
    secondArr.z = myCords[5];
    
    recast_ros::recast_path_planning srv;
    srv.request.startXYZ = firstArr;
    srv.request.goalXYZ = secondArr;

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

    return 0;
}

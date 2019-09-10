#include "ros/ros.h"
#include "recast_ros/AddObstacleSrv.h"
#include <pcl/common/io.h>
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_add_obstacles");

    ros::NodeHandle clientNode("~");
    ros::ServiceClient client = clientNode.serviceClient<recast_ros::AddObstacleSrv>("/recast_node/add_obstacle");

    geometry_msgs::Point firstArr;
    std::vector<double> myBuff(6, 1);
    std::vector<std::string> myMessage = {"x = ", "y = ", "z = ", "Radius = ", "Height = "};

    for (int i = 1; i < argc; i++)
    {
        myBuff[i - 1] = atof(argv[i]);
        ROS_INFO("%s%f", myMessage[i - 1].c_str(), myBuff[i - 1]);
    }

    firstArr.x = myBuff[0];
    firstArr.y = myBuff[1];
    firstArr.z = myBuff[2];

    float radius = myBuff[3];
    float height = myBuff[4];

    recast_ros::AddObstacleSrv srv;
    srv.request.position = firstArr;
    srv.request.radius = radius;
    srv.request.height = height;

    if (client.call(srv))
    {
        ROS_INFO("Obstacle is added, mesh is updated");
    }
    else
    {
        ROS_ERROR("Failed to add obstacle");
        return 1;
    }

    return 0;
}

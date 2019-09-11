//
// Copyright (c) 2019 Martim Brandão martim@robots.ox.ac.uk, Omer Burak Aladag aladagomer@sabanciuniv.edu
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

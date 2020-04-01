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

#include "ros/ros.h"
#include "recast_ros/RecastPathSrv.h"
#include <pcl/common/io.h>
#include <string>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_planning_service");

    ros::NodeHandle clientNode("~");
    ros::ServiceClient client = clientNode.serviceClient<recast_ros::RecastPathSrv>("/recast_node/plan_path");

    geometry_msgs::Point firstArr, secondArr;
    std::vector<double> myCords(6, 1);
    std::vector<std::string> myMessage = {"Start x = ", "Start y = ", "Start z = ", "Goal x = ", "Goal y = ", "Goal z = "};

    for (int i = 1; i < argc; i++)
    {
        myCords[i - 1] = atof(argv[i]);
        ROS_INFO("%s%f", myMessage[i - 1].c_str(), myCords[i - 1]);
    }

    firstArr.x = myCords[0];
    firstArr.y = myCords[1];
    firstArr.z = myCords[2];

    secondArr.x = myCords[3];
    secondArr.y = myCords[4];
    secondArr.z = myCords[5];

    recast_ros::RecastPathSrv srv;
    srv.request.startXYZ = firstArr;
    srv.request.goalXYZ = secondArr;

    if (client.call(srv))
    {
        std::vector<geometry_msgs::Point> path = srv.response.path;
        for (unsigned int i = 0; i < path.size(); i++)
        {
            ROS_INFO("Recevied Path[%d] = %f %f %f", i, path[i].x, path[i].y, path[i].z);
        }
    }
    else
    {
        ROS_ERROR("Failed to receive Path");
        return 1;
    }

    return 0;
}

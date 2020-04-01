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

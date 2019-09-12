//
// Copyright (c) 2019 Martim Brandão martim@robots.ox.ac.uk, Omer Burak Aladag aladagomer@sabanciuniv.edu, Ioannis Havoutis havoutis@robots.ox.ac.uk
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

#include "recast_ros/InputMeshSrv.h"
#include "recast_ros/RecastPlanner.h"
#include <pcl/common/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>

class InputMap
{
public:
    InputMap(ros::NodeHandle &node_handle) : node_handle_(node_handle)
    {
        // ros params
        node_handle_.getParam("path", path_);
        node_handle_.getParam("path_areas", path_areas_);
        node_handle_.param("path_service", path_service_, std::string("/recast_node/input_mesh"));

        //get reference point of the map
        node_handle_.getParam("reference_point_x", reference_point_.x);
        node_handle_.getParam("reference_point_y", reference_point_.y);
        node_handle_.getParam("reference_point_z", reference_point_.z);
    }
    void run()
    {
        // ros
        ros::ServiceClient client_recast = node_handle_.serviceClient<recast_ros::InputMeshSrv>(path_service_);
        recast_ros::InputMeshSrv srv;

        srv.request.reference_point.x = reference_point_.x;
        srv.request.reference_point.y = reference_point_.y;
        srv.request.reference_point.z = reference_point_.z;

        if (path_ == "")
        {
            ROS_ERROR("Couldn't find a valid map, check the launch file. Aborting...");
            return;
        }

        // load mesh
        bool loaded_mesh = pcl::io::loadPolygonFileOBJ(path_, pclMesh);
        if (loaded_mesh)
        {
            ROS_INFO("loaded OBJ file (%d polygons)", (int)pclMesh.polygons.size());
            pcl_conversions::fromPCL(pclMesh, srv.request.input_mesh);
        }
        else
        {
            ROS_ERROR("could not load OBJ file");
            return;
        }

        // load triangle labels (a.k.a. area types)
        std::vector<char> trilabels;
        bool loaded_areas = recast_ros::loadAreas(path_areas_, trilabels);
        if (loaded_areas)
        {
            ROS_INFO("loaded AREAS file (%d polygons)", (int)trilabels.size());
            srv.request.area_labels.resize(trilabels.size());
            for (size_t i = 0; i < trilabels.size(); i++)
                srv.request.area_labels[i] = trilabels[i];
        }
        else
        {
            ROS_WARN("could not load AREAS file... will assume all polygons are of area type 1");
            int n_tris = pclMesh.polygons.size();
            srv.request.area_labels.resize(n_tris);
            for (size_t i = 0; i < n_tris; i++)
                srv.request.area_labels[i] = 1;
        }

        if (client_recast.call(srv))
        {
            ROS_INFO("Map was sent");
        }
        else
        {
            ROS_ERROR("Failed to send map");
            return;
        }
    }

protected:
    ros::NodeHandle node_handle_;
    std::string path_service_;
    std::string path_;
    std::string path_areas_;
    pcl::PolygonMesh pclMesh;
    pcl::PointXYZ reference_point_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_input_mesh");
    ros::NodeHandle node_handle("~");
    InputMap InputMap(node_handle);

    InputMap.run();
    return 0;
}

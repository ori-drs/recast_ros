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
    }
    void run()
    {
        // ros
        ros::ServiceClient client_recast = node_handle_.serviceClient<recast_ros::InputMeshSrv>(path_service_);
        recast_ros::InputMeshSrv srv;

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
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_input_mesh");
    ros::NodeHandle node_handle("~");
    InputMap InputMap(node_handle);

    InputMap.run();
    return 0;
}

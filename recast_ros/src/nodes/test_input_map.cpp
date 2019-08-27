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
        std::string base = ros::package::getPath("recast_demos");
        node_handle_.param("path", path_, base + std::string("/data/map.obj"));
        node_handle_.param("path_areas", pathAreas_, base + std::string("/data/map.dat"));
        node_handle_.param("path_service", path_service_, std::string("/recast_node/input_mesh"));
    }
    void run()
    {
        // ros
        ros::ServiceClient client_recast = node_handle_.serviceClient<recast_ros::InputMeshSrv>(path_service_);
        recast_ros::InputMeshSrv srv;

        // load mesh
        bool loaded_mesh = pcl::io::loadPolygonFileOBJ(path_, pclMesh);
        if (loaded_mesh)
        {
            ROS_INFO("loaded OBJ file (%d polygons)", (int)pclMesh.polygons.size());
            pcl_conversions::fromPCL(pclMesh, srv.request.inputMesh);
        }
        else
        {
            ROS_ERROR("could not load OBJ file");
            return;
        }

        // load triangle labels (a.k.a. area types)
        std::vector<char> trilabels;
        bool loaded_areas = recast_ros::loadAreas(pathAreas_, trilabels);
        if (loaded_areas)
        {
            ROS_INFO("loaded AREAS file (%d polygons)", (int)trilabels.size());
            srv.request.areaLabels.resize(trilabels.size());
            for (size_t i = 0; i < trilabels.size(); i++)
                srv.request.areaLabels[i] = trilabels[i];
        }
        else
        {
            ROS_WARN("could not load AREAS file... will assume all polygons are of area type 1");
            int n_tris = pclMesh.polygons.size();
             srv.request.areaLabels.resize(n_tris);
            for (size_t i = 0; i < n_tris; i++)
                srv.request.areaLabels[i] = 1;
        }

        if (client_recast.call(srv))
        {
            ROS_INFO("Map is sent");
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
    std::string pathAreas_;
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

# recast_ros

This package is a ROS wrapper for [recastnavigation](https://github.com/recastnavigation/recastnavigation.git). It allows to build and use navigation meshes for path planning, where the meshes can have areas of different types (and costs).

If you use this in your research, please cite:

> Martim Brandao, Omer Burak Aladag, and Ioannis Havoutis, "**GaitMesh: controller-aware navigation meshes for long-range legged
locomotion planning in multi-layered environments**", in *ICRA2020* (submitted).


## Building

You need to download the original recastnavigation source code:
```
cd recast_ros/src
git clone https://github.com/recastnavigation/recastnavigation.git
```

### recast_demos

- Download https://drive.google.com/open?id=1i2Hkel-Nji3Zl0EdBjMIGzrxW9aALfRy
- Place it under the data folder.
- Then run the following:
```
roslaunch recast_demos gridmap_fsc_res5cm.launch
```

### Input mesh service to recast_node:

First run the demo to get an annotated mesh in the data folder (see recast_demos section above).
If you want to provide your own map as input, then set the "path" and "path_areas" parameters in recast_demos/launch/test_input_map.launch accordingly:

- "path" is the path to an .obj file with the mesh of your environment.
- "path_areas" (optional) is the path to a .dat file which is a binary-encoded sequence of char variables representing the area-type of each of the polygons in the .obj file, in the same order.

Then:

```
roslaunch recast_demos recast_node.launch
roslaunch recast_demos test_input_map.launch
```

### Reconfiguration of RecastPlanner

For the configuration of NavigationMesh and other parameters regarding to planner you can run:

```
rosrun rqt_reconfigure rqt_reconfigure
```

and set parameters to desired values.

Dynamic reconfiguring can also be turned off in the recast_node.launch file, in case you prefer to use launch-file parameters instead.

In this case you can run:

```
rosparam set /recast_node/param_name param_value
rosrun recast_ros test_update_parameters
```

This will update both configuration parameters and ros parameters. Mesh will be updated, if one of the mesh parameters is changed.

### Interactive RViz GUI:

This package comes with interactive user interface node (recast_node_interactive). 

You can use RViz InteractiveMarkers to,

- Set start position for the agent
- Set goal position for the agent
- Add obstacle(s) to desired position
- Remove all existing obstacles
- Delete specific obstacle(s)

All the changes made through recast_node_interactive will appear in recast_node RViz window.

#### WARNING:

While using the package, do not visualize InteractiveMarkers with Markers::LINE_LIST or Markers::LINE_STRIP, if you have a NVIDIA graphic card.
It causes RViz to segfault and crash.

Refer [here](https://github.com/ros-visualization/rviz/issues/1192) for details.

### Testing path planning service:

After input map is given to recast_node, for planning run:

```
rosrun recast_ros test_planning_service [start x] [start y] [start z] [goal x] [goal y] [goal z]
```

Other option is,

```
rosrun recast_ros test_planning_service_interactive
```

Use '2D Nav Goal' tool in RViz to give goal position to the agent. Start position is defined in recast_ros/src/nodes/test_planning_service_interactive.cpp


### Adding static obstacles service:

All obstacles assumed to be in cylindrical shape, to add obstacles to map run:

```
rosrun recast_ros test_add_obstacles [x pos] [y pos] [z pos] [radius] [height]
```

Each re-build operations removes all the obstacles present, another way to remove all obstacles is,

```
rosrun recast_ros test_remove_all_obstacles
```

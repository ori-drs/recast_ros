# recast_ros

## recast_ros

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

First run the demo to get the FSC mesh (see recast_demos section above).
If you have another map as input, put .obj file path and .dat file path ( .dat file is for area types, this is optional) into recast_demos/launch/test_input_map.launch
Then:

```
roslaunch recast_demos recast_node.launch
roslaunch recast_demos test_input_map.launch
```

For configuration of NavigationMesh run:

```
rosrun rqt_reconfigure rqt_reconfigure
```

Set respective parameters Recast/Detour and RecastPlanner to desired values within limitations.

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

**Each re-build operations removes all the obstacles present, another way to remove all obstacles is,

```
rosrun recast_ros test_remove_all_obstacles
```
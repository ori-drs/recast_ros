# recast_ros

## recast_ros

You need to download the original recastnavigation source code:
```
cd recast_ros/src
git clone https://github.com/recastnavigation/recastnavigation.git
```

### Testing path planning service:

First run the demo to get the FSC mesh (see recast_demos section below).
Then:

```
roslaunch ....
rosrun .... [start x] [start y] [start z] [] [] []
```

## recast_demos

- Download https://drive.google.com/open?id=1i2Hkel-Nji3Zl0EdBjMIGzrxW9aALfRy
- Place it under the data folder.
- Then run the following:
```
roslaunch recast_demos gridmap_fsc_res5cm.launch
```

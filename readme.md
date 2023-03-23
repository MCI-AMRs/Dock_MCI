# ROS2_MCI

## Launch NAV sync
```
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=sync
```

## Launch NAV
```
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=map_MCI.yaml
```

## start sync slam (Mapping)
```
ros2 launch turtlebot4_navigation slam_sync.launch.py
```

## launch RVIZ
```
ros2 launch turtlebot4_viz view_robot.launch.py
```

## save the created map
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```

## cpy map to RPI
```
scp map.yaml ubuntu@192.168.50.185: dir/etc/
```

## run QR Docking action
```
ros2 action send_goal /dock_turtle action_interfaces/action/Dock "{goal: true}"
```

## setup ssh publick key
```
https://kb.iu.edu/d/aews
```

## Links papers
```
https://openlib.tugraz.at/download.php?id=5f6af343ccab6&location=browse
https://arxiv.org/pdf/2104.12954.pdf
```

## Cam setup path
```
/opt/ros/galactic/share/turtlebot4_bringup/launch/oakd.launch.py
```

# Cam depthai installation
```
sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
```

# Oak-d CameraMatrix and DisCoeffs
```
nano /opt/ros/galactic/share/depthai_examples/params/camera/color.yaml
```
#### During development I faced some problems, here are some of them:

1. In Trying to run `gui/main.py` failed, because of missing `geographic_msg` plugin, I fixed it with <br> `sudo apt-get install ros-noetic-geographic_msgs`

2. After first run of gazebo, the urdf model failed to spawn, and the entire simulation was broken. 
Fixed, by clearing all gazebo instances
```$killall -9 gzserver & killall -9 gzclient & killall -9 gazebo```

killall -9 gzserver & killall -9 gzclient & killall -9 gazebo

3. Create c++ control plugin for Gazebo simulation
https://classic.gazebosim.org/tutorials?tut=guided_i5



export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Robotics_ws/control_plugin/build

follow this tutorial: https://github.com/pal-robotics/gazebo_ros_link_attacher/
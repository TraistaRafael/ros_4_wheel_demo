
#### The scope of this project is to provide a Gazebo 4-wheel simulation and communication whith a ROS node.

![Alt text](ros_demo.png)

## Build & Run

### 1. Prerequisites:
- ROS Noetic
- Gazebo

### 2. Simulation 
- clone repository to `~/Robotics_ws`
- Build control plugin
```
cd ~/Robotics_ws/control_plugin
mkdir build
cd build
cmake ..
make
```
- Build & run simulation
```
cd ~/Robotics_ws
catkin_make
source devel/setup.sh
source control_plugin/build/devel/setup.sh
roslaunch atom world.launch
```

Note: If you want to restart the simulation, or had any gazebo instance previously open, you may have to close all related processes by:
```
killall -9 gzserver & killall -9 gzclient & killall -9 gazebo
```

- Run GUI map
```
cd ~/Robotics_ws/src/gui
python3 -m pip install -r requirements.txt
python 3 main.py
```

### 3. Development notes 

1. Trying to run `gui/main.py` failed, because of missing `geographic_msg` plugin, I fixed it with <br> `sudo apt-get install ros-noetic-geographic_msgs`

2. After first run of gazebo, the urdf model failed to spawn, and the entire simulation was broken. 
Fixed, by clearing all gazebo instances
```killall -9 gzserver & killall -9 gzclient & killall -9 gazebo```

3. In order to create the C++ control plugin for Gazebo I followed this sources & tutorials:
- https://classic.gazebosim.org/tutorials?tut=guided_i5
- https://github.com/pal-robotics/gazebo_ros_link_attacher/
- https://github.com/ros-simulation/gazebo_ros_pkgs/blob/f9e1a4607842afa5888ef01de31cd64a1e3e297f/gazebo_plugins/src/gazebo_ros_skid_steer_drive.cpp#L70

4. The initial attempts to compile the plugin failed, the problems were fixed after imlementing in `CMakeLists.txt` the missing gazebo & catkin dependencies. Also I had to create `control_plugin/package.xml`

5. My C++ plugin was not communicating with ROS, althought `rqt_graph` showed everything correct. After investigations, I found that the subscription thread was not actually running. I was missing this part:
```
// start custom queue for diff drive
callback_queue_thread_ =
    boost::thread(boost::bind(&ControlPlugin::QueueThread, this));

// listen to the update event (broadcast every simulation iteration)
update_connection_ =
    event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ControlPlugin::UpdateChild, this));
```

6. Initially, I was not able to see if the wheels are actually rotating, so I added some yellow indicators on wheels.

7. At some point, the robot was moving almost ok, but the left-right direction was reversed relative to GUI map. The solution was to update my conversion functions, to reverse langitude axis.  
For more details please check the following:
```
sensor_msgs::NavSatFix ControlPlugin::GazeboPosToGeoLoc(ignition::math::Vector3d  gazebo_pos)
ignition::math::Vector3d ControlPlugin::GeoLocToGazeboPos(sensor_msgs::NavSatFix geo_loc)
```

# Base repository notes

## Autonomous Navigation

Refer to doc here: [Navigation](/docs/Navigation.md)
  
## Author

👤 Harsh Mittal

Twitter: [@harshmittal2210](https://twitter.com/harshmittal2210)
Github: [@harshmittal2210](https://github.com/harshmittal2210)
Website: [harshmittal.co.in](http://harshmittal.co.in)
  
## 🤝 Contributing

Contributions, issues and feature requests are welcome!

## Show your support

Give a ⭐️ if you think this project is awesome!

## 📝 License

Copyright © 2021 [Harsh Mittal](https://github.com/harshmittal2210).
This project is [Apache License](https://github.com/harshmittal2210/Robotics_ws/blob/main/LICENSE) licensed.

Lidar Publisher Translation Codelet
====================================

This codelet aims to take fltascan lidar information from Isaac Sim and translate them to ROS laser scan message published on a ROS topic.

Isaac Codelet Name
------------------
`isaac::rosbridge::LidarRosBridge`

Input
-----
Isaac `FlatscanProto` received whenever simulated flattened lidar information for a robot is available.
Proto is received on `lidar_scan` channel.

Output
------
ROS sensor_msgs/Laserscan Message published on topic defined by ISAAC_PARAM lidar_ros_topic_name.

Isaac parameters
----------------

`lidar_ros_topic_name` - the name of the ROS topic the lidar is published on

`lidar_ros_frame_name` - name of the frame used in ROS corresponding to the lidar frame where laser scan points are added relative to

Setup
-----
If you have followed all steps of the main setup instructions for this repo then nothing further is required. 

This codelet itself does not require any update of the ROS package, but the example application also runs the odom and robot_tf codelets. Please see their documentation to install what is required for them to run the applications.

Example App
-----------
The example application is built upon the basic carter_sim_joystick application where the robot is moved around it's environment using joystick controls. All this application does is publish the flatscan lidar information into ROS. In this app we use previously build odom_publisher and tf_robot_publisher codelets to provide tf context for the laser scan and enable visualization in rviz.

For the purpose of this application we simply view the lidar laser scan in rviz to confirm this is operating as expected.

**WARNING** It is important that you check the lidar is operating as expected. We have had some instances of inverted lidar scans (left and right switched) for as-yet unknown reasons. if you experience inverted lidar scans uncomment the appropriate line in `ros_bridge_lidar.cpp`

### Running the App ###
Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac SDK folder:  
```
bazel run //isaac_ros_bridge_codelets/lidar_publisher:carter_sim_ros_lidar_pub -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
```

Terminal 3:
```
roscore
``` 

Terminal 4:
```
rviz
```

With rviz running, set the fixed frame to "odom", visualize the transform by adding tf, and visualize the laser scan by adding LaserScan.



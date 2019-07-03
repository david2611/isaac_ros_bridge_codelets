Twist Subscriber Translation Codelet
====================================

This codelet aims to take Twist messages from ROS and translate them to StateProto messages used by Isaac to move 
robots with a differential base system. This assumes the Twist message has only single linear and angular velocity
components.

Isaac Codelet Name
------------------
`isaac::rosbridge::MovementRosBridge`

Input
-----
ROS `geometry_msg/Twist` messages with a single linear velocity `linear.x` and a single angular velocity `angular.z`.

Output
------
Isaac StateProto transmitted along the channel `base_cmd`.

Isaac parameters
----------------

`subscriber_queue_size` - the ROS subscriber queue depth

`subscriber_channel_name` - name of the ROS topic codelet subscribes to

Setup
-----
If you have followed steps 1. and 2. of the main setup instructions for this repo then nothing further is required.

Example App
-----------
The provided example is equivalent to the Isaac carter_sim_joystick application where you are able to move the carter robot
within a simulated environment using a joystick.

However, the principle difference is that controlling the robot is done using ROS Twist messages generated through ROS 
joystick teleop nodes rather than the Isaac joystick libraries to show that we can translate ROS Twist messages for Isaac.

### Running the App ###
Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac SDK folder:  
```
bazel run //isaac_ros_bridge_codelets/twist_subscriber:carter_sim_ros_move -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
```

Terminal 3:
```
roscore
```

Terminal 4:
```
rosrun joy joy_node
```

Terminal 5:
```
rosrun teleop_twist_joy teleop_node
```
**Note:** you may need to reconfigure teleop_node to match your joystick config (e.g. `_axis_angular:=2`) 

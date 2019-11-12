Odometry Publisher Translation Codelet
====================================

This codelet aims to take odometry information from Isaac Sim and translate them to ROS Odometry message published on a ROS topic.
This also publishes the frame information of odometry frame and base_link (robot) frame. 


Isaac Codelet Name
------------------
`isaac::rosbridge::OdometryRosBridge`

Input
-----
Isaac `Odometry2Proto` received whenever simulated odometry information for a robot is available.
Proto is received on `odometry` channel.

Output
------
ROS nav_msgs/Odometry Message published on topic defined by ISAAC_PARAM ros_publisher_channel_name.
Transform broadcaster also sends out transform of odometry frame and robot (base_link) frame.

Isaac parameters
----------------

`publisher_queue_size` - the ROS publisher queue depth

`publisher_channel_name` - name of the ROS topic codelet publishes to

`ros_header_frame` - name of the header frame used in ROS describing robot initial origin(default "odom")

`ros_child_frame` - name of the child frame used in ROS describing the robot frame (default "base_link")

Setup
-----
If you have followed all steps of the main setup instructions for this repo then nothing further is required. If ROS packages are not updated, here is the instructions for installing the ROS packages required for this translation codelet.

This codelet requires use of the `tf` package not available in the base Isaac installation. If you did not complete step 3. of the main setup instructions you will need to include these packages in your Isaac ROS setup.

To include these packages complete the following:
1. Update `PACKAGES` parameter in `engine/build/scripts/ros_package_generation.sh` of your isaac_sdk installation 
to read:
    ```
    PACKAGES="roscpp rospy actionlib_msgs control_msgs diagnostic_msgs geometry_msgs
     map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs stereo_msgs
     tf2_geometry_msgs tf2_msgs trajectory_msgs visualization_msgs tf"
    ```

2. Run `engine/build/scripts/ros_package_generation.sh <distro> <package_name>`
    * `<distro>` - ROS distribution used (should be `melodic`)
    * `<package_name>` - name you want to give your package

3. untar the output tarball file created to an appropriate locatione (e.g. ros_packages)

4. Update the `third_party/ros.bzl` file in your Isaac installation.
    * Add `"isaac_local_repository"` to the first load command
    * Comment out the following lines:
    ```
    isaac_new_http_archive(
        name = "isaac_ros_bridge_x86_64",
        build_file = clean_dep("//third_party:ros.BUILD"),
        sha256 = "9795b242c2636c2545c2c581037e9a27091e922faec9617784ae5705989f6e17",
        url = "https://developer.nvidia.com/isaac/download/third_party/ros-melodic-x86_64-20190327-tar-gz",
        type = "tar.gz",
        # We only use packages under BSD licenses from this list.
        licenses = ["https://docs.ros.org/diamondback/api/licenses.html"],
    )
    ```
    * Underneath these lines add the following:
    ```
    isaac_new_local_repository(
        name = "isaac_ros_bridge_x86_64",
        path = <pkg_path>,
        build_file = clean_dep("//third_party:ros.BUILD"),      
        # We only use packages under BSD licenses from this list.
        licenses = ["https://docs.ros.org/diamondback/api/licenses.html"],
    )
    ```
    where `<pkg_path>` is a string holding the full path to your extracted package.

Example App
-----------
The example application is built upon the basic carter_sim_joystick application where the robot is moved around it's environment using joystick controls.
All this application does is publish the odometry and frame information into ROS.

For the purpose of this application we simply view the ROS topic output with rostopic echo and rviz to confirm this is operating as expected.

### Running the App ###
Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac SDK folder:  
```
bazel run //isaac_ros_bridge_codelets/odom_publisher:carter_sim_ros_odom_pub -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
```

Terminal 3:
```
roscore
``` 

Terminal 4:
```
rostopic echo /isaac_odometry
```

Terminal 5:
```
rviz
```

With rviz running, set the fixed frame to "odom" and visualize the transform by adding tf.



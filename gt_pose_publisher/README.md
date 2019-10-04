GT Pose Difference Publisher Translation Codelet
=============================================

This codelet aims to publish the difference in pose between the ground-truth location of the robot and the robot's estimated location based on odometry.

Unlike other translation codelets, this is comprised of two parts.
1. Codelet to calculate the gt location of a robot w.r.t. the ground-truth pose of where the robot was initialized (initial pose defined outside the codelet using PoseInitializer)
2. Codelet to calculate the difference between the estimated of the robot w.r.t. where robot started and the ground-truth pose of the robot w.r.t. where the robot started (odomTrobot vs. gt_odomTgt_robot) and publish this relationship to ROS.

WARNINGS AND ASSUMPTIONS
------------------------
1. Assumes currently that the RigidBodyGroup is using Isaac-based coordinate frames (pose centred at the base of the robot). Don't know for certain if this is true or they are slightly Unreal-based (pose centred in the middle of the robot).
2. There are some assumptions made about naming conventions in the Isaac PoseTree and ROS TF. Most should be adjustable as IsaacParameters but need to double check.
3. The first stage is probably not necessary but I found it useful to understand what was going on within Isaac with ground-truth poses.
    * To use this stage effectively, you will need to define a PoseInitializer to define the gt_robot_init and gt_world poses.
    * These poses should be based upon what is written in carter_full.config.json and would need to be updated for any change of map.


Isaac Codelet Names
------------------
* `isaac::GTPosePublisher`
* `isaac::rosbridge::GTPoseDiffTFRosBridge`

Input
-----
GTPosePublisher:
* `gt_poses` channel receives a RigidBody3GroupProto containing the RigidBodies for all defined objects/robots in the scene? Exactly which object/robot used is defined by the `rigid_body_robot_name` IsaacParameter.

GTPoseDiffTFRosBridge: None

Output
------
ROS transform broadcast difference between gt_robot and robot (difference between map and odom)?

Isaac parameters
----------------
GTPosePublisher:
* `isaac_gt_robot_name` - name given to the ground-truth robot in the Isaac PoseTree (default "gt_robot")
* `rigid_body_robot_name` - name of the robot in the list of rigid bodies. Should be defined in carter_full.config.json (e.g. "carter_1")
* `gt_world_frame_name` - name given to the ground-truth world origin in the Isaac PoseTree  (default "gt_world")

GTPoseDiffTFRosBridge:
* `isaac_robot_name` - name of the robot frame we care about as it appears in the Isaac PoseTree (default "robot")
* `isaac_odom_name` - name of the odometry frame for the robot as defined in the Isaac PoseTree  (where the robot thinks it started from) (default "odom")
* `isaac_gt_robot_name` - name of the gt robot frame we care about as it appears in the Isaac PoseTree (default "gt_robot")
* `isaac_gt_odom_name` - name of the ground-truth odometry frame for the robot as defined in the Isaac PoseTree (initialization position should be defined in bridge_config and matched in a PoseInitializer)(default "gt_robot_init")
* `ros_parent_frame` - name of the parent (header) frame for the published diff transform (note that despite calculating the difference between robot poses, we use map and odom difference by default which are the same)? (default "map")
* `ros_child_frame` - name f the child frame for the published diff transform (default "odom")
* `print_debug` - boolean flag for whether debugging print messages should be printed to screen showing various transform values. (default false)
* `tick_period` - frequency of ticks.

Setup
-----
If you have followed steps 1., 2., and 3. of the main setup instructions for this repo then nothing further is required.

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
All this application does is publish the odometry and frame information into ROS. In this app we use previously built odom_publisher, and tf_robot_publisher codelets to provide the full tf we would want output by the simulator system.

For the purpose of this application we simply view the ROS topic output with rviz to confirm this is operating as expected.

We should see a tf visualization with the map (gt_robot_init) robot, odom (where robot thinks robot_init is), two cameras, and a lidar.

### WARNING ###
If you change maps, you will need to update pose initializer information in the app.json.

### Running the App ###
Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac SDK folder:  
```
bazel run //isaac_ros_bridge_codelets/gt_pose_publisher:carter_sim_ros_gt_pose_pub -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
```

Terminal 3:
```
roscore
``` 

Terminal 4:
```
rviz
``` 

Isaac - ROS Translation Codelets
=================================
This repo contains codelets created to transfer information between ROS and Isaac. 
Each folder is fairly self-contained with its own Codelet publishing/subscribing one type of message to/from ROS.
Each folder should contain the codelet source code (.hpp and .cpp files) a bazel BUILD file, and an example .app.json file.
Most codelets work with more than is supplied in the base version of ROS supplied with Isaac and where this occurs is specified.
Example apps should provide some meaningful, easily debugged test example that proves translation is working properly (note some codelets use other existing translation codelets). 

Note that code here was developed whilst learning how Isaac and ROS communicate and may have "unnecessary" comments and general flaws. 
It is encouraged that contributors maintain a high-level of commenting and clarity to ensure easy use and understanding by others.

**Currently assumes repository is stored in the root Isaac SDK folder. If not see setup.**

**Designed to operate within Isaac SDK 2019.2**

### Notes for contributors ###
Anyone who adds a new translation codelet should try to follow the structure of already existing codelets, being sure to
provide an example app that demonstrates the codelet and provide appropriate commenting.

Contributors should also provide documentation here and within their codelet's subfolder explaining the contents, how
it works, what the demonstration app does, and any extra requirements needed.

If your codelet requires ROS packages not included by default in Isaac's ROS setup, add instructions for installing
and utilising the ROS package for Isaac in the subfolder's README.

If these extra packages are not included in the Update ROS Packages section in the Setup, add them to the list of 
packages as appropriate.

Setup
=====
Assuming you have successfully installed Isaac_sdk and Isaac_sim, in order to run any of the codelet example code 
provided you must complete the following steps:

### Update carter_sim BUILD file ###

Add the following to the bazel BUILD file in  your isaac_sdk apps/carter/carter_sim/ folder to enable use of carter_sim config files outside of the carter_sim project:
```
filegroup(
    name = "carter_setup_files",
    data = [
            "carter.config.json",
            "carter.graph.json"
           ],
    visibility = ["//visibility:public"],
)
    
filegroup(
    name = "navigation_setup_files",
    data = [
            "navigation.config.json",
            "navigation.graph.json"
           ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "joystick_setup_files",
    data = [
        "joystick.config.json",
        "joystick.graph.json"
    ],
    visibility = ["//visibility:public"],
)
```
### Update path to codelet sub-folders ###
If you have not saved the repo your base Isaac SDK folder, you must change the path to the codelet sub-folders in the repo within those sub-folders' .app.json and BUILD files.

For example if isaac_ros_bridge_codelets is saved in packages in Isaac SDK folder line 18 of 
img_publisher/carter_sim_ros_img_pub.app.json changes from 
    `"//isaac_ros_bridge_codelets/img_publisher:ros_img_pub_components"` 
    to
    `"//packages/isaac_ros_bridge_codelets/img_publisher:ros_img_pub_components"`
    
### (Potentially Optional) Update ROS Packages ###
Most codelets require changing the base ROS packages available to Isaac. 
If different packages are required the specific requirements will be outlined for that specific codelet.
To have the ros package requirements required for all translation codelets used so far do the following:

1. Update `PACKAGES` parameter in `engine/build/scripts/ros_package_generation.sh` of your isaac_sdk installation 
to read:
    ```
    PACKAGES="roscpp rospy actionlib_msgs control_msgs diagnostic_msgs geometry_msgs
     map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs stereo_msgs
     tf2_geometry_msgs tf2_msgs trajectory_msgs visualization_msgs cv_bridge image_transport, tf"
    ```

2. Run `engine/build/scripts/ros_package_generation.sh <distro> <package_name>`
    * `<distro>` - ROS distribution used (should be `melodic`)
    * `<package_name>` - name you want to give your package
        * NOTE: do not use this to define a subdirectory to save the package into. Assume package will be created within the folder the script call is made otherwise the script breaks.

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
 

Current Translation Codelets
============================
Each folder's content relates to a different translation codelet. Here should be a brief high-level description of the 
codelet. Further information should be provided in separate README for the codelet containing all parameters that can and would be changed for varying configurations.
Example App code assumes default storage location for this repository and Isaac Sim setup completed.

depth_img_publisher
-------------
**Purpose:** Enable Depth images from Isaac Sim to be published as ROS Image messages.

**Message Conversion:**  Isaac DepthCameraProto -> ROS Image Message

**Running Example App:**

Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac sdk folder:  
```
bazel run //isaac_ros_bridge_codelets/depth_img_publisher:carter_sim_ros_depth_img_pub -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
```
Terminal 3:
```
roscore
```

Terminal 4:
```
rqt_image_view
```

**Extra ROS Requirements:**  `cv_bridge` and  `image_transport` packages

gt_pose_publisher
------------------
**Purpose:** Transmit the ground-truth pose information form Isaac simulator to ROS. Sends the difference between estimated and true robot pose alongside ground-truth world and odom pose.

**Message Conversion:**  Isaac RigidBody3Proto and Pose3d -> ROS geometry_msgs::TransformStamped messages

**Running Example App:**

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

Terminal 4 (optional) within or referencing object_gt_visualization folder:
```
python visualize_gt_rviz.py --gt_file <gt_pose_file> --ros_topic <topic_name> --header_frame <world_frame_name>`
```

 where `<gt_pose_file>` is the .json output from `gen_challenge_gt.py`, `<topic_name>` is the name of the ros topic that you publish your gt object markers to, and `world_frame_name` is the name of the world frame in ROS that corresponds to the world frame the objects were originally mapped with.

Terminal 5:
```
rviz
``` 
 
 Note, you will need to move the robot with the joystick through Isaac and set up RViz appropriately to view all the components. Current version uses odom_publisher as well

**Extra ROS Requirements:**  `tf` package

img_publisher
-------------
**Purpose:** Enable RGB images from Isaac Sim to be published as ROS Image messages.

**Message Conversion:**  Isaac ColorCameraProto -> ROS Image Message

**Running Example App:**

Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac sdk folder:  
```
bazel run //isaac_ros_bridge_codelets/img_publisher:carter_sim_ros_img_pub -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
```
Terminal 3:
```
roscore
```

Terminal 4:
```
rqt_image_view
```

**Extra ROS Requirements:**  `cv_bridge` and  `image_transport` packages

lidar_publisher
------------------
**Purpose:** Transmit lidar information form Isaac to ROS.

**Message Conversion:**  Isaac FlatscanProto -> ROS sensor_msgs::Laserscan messages

**Running Example App:**

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

**Extra ROS Requirements:**  `tf` package (only for the example app)

odom_publisher
------------------
**Purpose:** Transmit the odometry information (including assumed poses) form Isaac to ROS.

**Message Conversion:**  Isaac Odometry2Proto -> ROS geometry_msgs::TransformStamped and ROS nav_msgs::Odometry messages

**Running Example App:**

Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac SDK folder:  
```
bazel run //isaac_ros_bridge_codelets/twist_subscriber:carter_sim_ros_odom_pub-- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
```

Terminal 3:
```
roscore
```

Terminal 4:
```
rviz
```
 
 Note, you will need to move the robot with the joystick through Isaac and set up RViz appropriately to view all the components. Current version uses odom_publisher as well

**Extra ROS Requirements:**  `tf` package

robot_tf_publisher
------------------
**Purpose:** Transmit the relative pose information of a robot in Isaac within the Isaac PoseTree to ROS tf frames.

**Message Conversion:**  Multiple Isaac Pose3d from Isaac PoseTree-> ROS tf

**Running Example App:**

Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
```
where `<isaac_sdk_path>` is the path to your Isaac SDK installation.

Terminal 2 within Isaac SDK folder:  
```
bazel run //isaac_ros_bridge_codelets/twist_subscriber:carter_sim_ros_robot_tf_pub-- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
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
 
 Note, you will need to move the robot with the joystick through Isaac and set up RViz appropriately to view all the components.

**Extra ROS Requirements:**  `tf` package

twist_subscriber
----------------
**Purpose:** Translate ROS Twist messages to enable movement of simulated Isaac robots. 
Currently assumes Twist message contains a single linear velocity and a single angular velocity reading.

**Message Conversion:**  ROS geometry_msg/Twist -> Isaac StateProto

**Running Example App:**

Terminal 1 within Isaac Sim folder:
```
./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P vulkan -isaac_sim_config_json="<isaac_sdk_path>/apps/carter/carter_sim/bridge_config/carter_full.json"
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
Note: you may need to reconfigure teleop_node to match your joystick config (e.g. `_axis_angular:=2`) 

**Extra ROS Requirements:**  None
Isaac - ROS Translation Codelets
=================================
This repo contains codelets created to transfer information between ROS and Isaac. 
Each folder is self-contained with its own Codelet publishing/subscribing one type of message to/from ROS.
Each folder should contain the codelet source code (.hpp and .cpp files) a bazel BUILD file, and an example .app.json file.
Codelets should be able to work with the base version of ROS that comes with Isaac unless otherwise specified.
Example apps should provide some meaningful, easily debugged test example that proves translation is working properly. 

Note that code here is likely "over-commented" as it was developed whilst learning how Isaac and ROS communicate. 
It is encouraged that contributors maintain a high-level of commenting and clarity to ensure easy use by others.

**Currently assumes repository is stored in the root Isaac SDK folder. If not see setup.**

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
```
### Update path to codelet sub-folders ###
If you have not saved the repo your base Isaac SDK folder, you must change the path to the codelet sub-folders in the repo within those sub-folders' .app.json and BUILD files.

For example if ros_bridge_codelets is saved in packages in Isaac SDK folder line 18 of 
img_publisher/carter_sim_ros_img_pub.app.json changes from 
    `"//isaac_ros_bridge_codelets/img_publisher:ros_img_pub_components"` 
    to
    `"//packages/isaac_ros_bridge_codelets/img_publisher:ros_img_pub_components"`
    
### (Optional) Update ROS Packages ###
Most codelets should not require changing the base ROS packages available to Isaac. 
If different packages are required the specific requirements will be outlined for that specific codelet.
To have the ros package requirements required for all translation codelets used so far do the following:

1. Update `PACKAGES` parameter in `engine/build/scripts/ros_package_generation.sh` of your isaac_sdk installation 
to read:
    ```
    PACKAGES="roscpp rospy actionlib_msgs control_msgs diagnostic_msgs geometry_msgs
     map_msgs nav_msgs pcl_msgs sensor_msgs shape_msgs std_msgs stereo_msgs
     tf2_geometry_msgs tf2_msgs trajectory_msgs visualization_msgs cv_bridge image_transport"
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
 

Current Translation Codelets
============================
Each folder's content relates to a different translation codelet. Here should be a brief high-level description of the 
codelet. Further information should be provided in separate README for the codelet.
Example App code assumes default storage location for this repository and Isaac Sim setup completed.

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
rqt_image_view
```

**Extra ROS Requirements:**  `cv_bridge` and  `image_transport` packages

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




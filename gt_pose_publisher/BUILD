load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module", "isaac_pkg")

isaac_app(
    name = "carter_sim_ros_gt_pose_pub",
    app_json_file = "carter_sim_ros_gt_pose_pub.app.json",
    data = [
        "//apps/assets/maps",
        "//packages/map:libmap_module.so",
        "//apps/carter/carter_sim:carter_setup_files",
        "//apps/carter/carter_sim:navigation_setup_files",
        "//apps/carter/carter_sim:joystick_setup_files",
    ],
    modules = [
        "navigation",
        "perception",
        "planner",
        "viewers",
        "flatsim",
        "sensors:joystick",
        "//isaac_ros_bridge_codelets/gt_pose_publisher:isaac_gt_pose_pub_components",
        "//isaac_ros_bridge_codelets/gt_pose_publisher:ros_gt_pose_pub_components",
        "//isaac_ros_bridge_codelets/odom_publisher:ros_odom_pub_components",
        "//isaac_ros_bridge_codelets/robot_tf_publisher:ros_robot_tf_pub_components"
        
    ],
)

isaac_cc_module(
  name = "ros_gt_pose_pub_components",
  srcs = ["ros_bridge_gt_pose_diff.cpp"],
  hdrs = ["ros_bridge_gt_pose_diff.hpp"],
  visibility = ["//visibility:public"],
    deps = [
        "//third_party:ros"
    ]
)

isaac_cc_module(
  name = "isaac_gt_pose_pub_components",
  srcs = ["isaac_gt_pose.cpp"],
  hdrs = ["isaac_gt_pose.hpp"],
  visibility = ["//visibility:public"],
)

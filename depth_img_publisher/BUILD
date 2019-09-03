load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module", "isaac_pkg")

isaac_app(
    name = "carter_sim_ros_depth_img_pub",
    app_json_file = "carter_sim_ros_depth_img_pub.app.json",
    data = [
        "//apps/assets/maps",
        "//packages/map:libmap_module.so",
        "//apps/carter/carter_sim:carter_setup_files",
        "//apps/carter/carter_sim:navigation_setup_files",
    ],
    modules = [
        "navigation",
        "perception",
        "planner",
        "viewers",
        "flatsim",
        "//isaac_ros_bridge_codelets/depth_img_publisher:ros_depth_img_pub_components",
        
    ],
)

isaac_cc_module(
  name = "ros_depth_img_pub_components",
  srcs = ["ros_bridge_depth_img.cpp"],
  hdrs = ["ros_bridge_depth_img.hpp"],
  visibility = ["//visibility:public"],
    deps = [
        "//third_party:opencv",
        "//third_party:ros"
    ]
)

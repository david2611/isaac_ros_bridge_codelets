load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module", "isaac_pkg")

isaac_app(
    name = "carter_sim_ros_move",
    app_json_file = "carter_sim_ros_move.app.json",
    data = [
        "//apps/assets/maps",
        "//packages/map:libmap_module.so",
        "//apps/carter_sim:carter_setup_files",
        "//apps/carter_sim:navigation_setup_files",
    ],
    modules = [
        "navigation",
        "perception",
        "planner",
        "viewers",
        "flatsim",
        "//isaac_ros_bridge_codelets/twist_subscriber:ros_twist_sub_components"
    ],
)

isaac_cc_module(
  name = "ros_twist_sub_components",
  srcs = ["ros_bridge_move.cpp"],
  hdrs = ["ros_bridge_move.hpp"],
  visibility = ["//visibility:public"],
    deps = [
        "//third_party:ros",
        "//messages/state:differential_base",
        "//engine/gems/state:io"
    ]
)

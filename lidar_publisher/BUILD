load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module", "isaac_pkg")

isaac_cc_module(
    name = "ros_lidar_pub_components",
    srcs = ["lidar_publisher.cc"],
    hdrs = ["lidar_publisher.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:ros",
        ]
    )

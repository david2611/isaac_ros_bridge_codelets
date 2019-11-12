#include "ros_bridge_lidar.hpp"

#include <algorithm>
#include <fstream>

// #include "helpers.hh"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

namespace isaac {
namespace rosbridge {

constexpr double LIDAR_FOV = 2 * M_PI;

inline double wrap_angle_to_pi(const double angle) {
  // Returns an angle between -PI & +PI in an inefficient way...
  double a = angle;
  while (a <= -M_PI) a += 2 * M_PI;
  while (a >= M_PI) a -= 2 * M_PI;
  return a;
}

struct LidarRosBridge::RosData {
  ros::NodeHandle nh;
  ros::Publisher pub;
};

void LidarRosBridge::start() {
  // Start our monolothic ROS node if it doesn't already exist
  if (!ros::isInitialized())
    ros::init(ros::M_string(), "benchbot_simulator",
              ros::init_options::NoSigintHandler);

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->pub = ros_data_->nh.advertise<sensor_msgs::LaserScan>(
      get_lidar_ros_topic_name(), 2);

  // Configure the codelet to only tick when we receive a new camera message
  // from the simulator
  tickOnMessage(rx_lidar_scan());
}

void LidarRosBridge::stop() {
  ros_data_->pub.shutdown();
  ros_data_ = nullptr;
}

void LidarRosBridge::tick() {
  if (ros::ok()) {
    // Received a message, cache time ASAP
    ros::Time msg_time = ros::Time::now();

    // Turn the Isaac message into a ROS message
    auto lidar_proto = rx_lidar_scan().getProto();
    sensor_msgs::LaserScan lidar_ros;
    lidar_ros.header.stamp = msg_time;
    lidar_ros.header.frame_id = get_lidar_ros_frame_name();
    lidar_ros.angle_min = *(lidar_proto.getAngles().end() - 1);
    lidar_ros.angle_max = *lidar_proto.getAngles().begin();
    lidar_ros.angle_increment =
        LIDAR_FOV / double(lidar_proto.getAngles().size());
    lidar_ros.range_min = lidar_proto.getInvalidRangeThreshold();
    lidar_ros.range_max = lidar_proto.getOutOfRangeThreshold();
    for (const double &r : lidar_proto.getRanges())
      lidar_ros.ranges.push_back(r);

    // reverse needed for Ben setup. Not sure why yet.
    // std::reverse(lidar_ros.ranges.begin(), lidar_ros.ranges.end());

    // Publish the ROS LaserScan message
    ros_data_->pub.publish(lidar_ros);
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

}  // namespace rosbrdige
}  // namespace isaac

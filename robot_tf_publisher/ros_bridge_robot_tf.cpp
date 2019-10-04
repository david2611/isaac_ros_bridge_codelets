#include "ros_bridge_robot_tf.hpp"


#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

namespace isaac {
namespace rosbridge {

void print_pose(Pose3d pose){ 
  auto translation = pose.translation;
  auto rpy = pose.rotation.eulerAnglesRPY();
  auto quaternion = pose.rotation.quaternion();
  // print translation
  std::cout << "Translation: {X: " << translation[0] << ", Y: " << translation[1] ;
  std::cout << ", Z: " << translation[2] << "}\n";
  // print rotation roll pitch yaw
  std::cout << "RPY: {Roll: " << rpy[0] << ", Pitch: " << rpy[1] << ", Yaw: ";
  std::cout << rpy[2] << "}\n";
  // print rotation quaternion
  std::cout << "Quaternion: {W: " << quaternion.w() << ", X: " << quaternion.x() << ", Y: ";
  std::cout << quaternion.y() << ", Z: " << quaternion.z() << "}\n";
}

// Internal struct for holding the ROS node handle and the publisher/ImageTransport components
struct RobotTFRosBridge::RosRobotTFData {
  ros::NodeHandle node;
  tf::TransformBroadcaster tf_broadcaster;
};

void RobotTFRosBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // Initialise data struct and define what topic node is publishing on for ROS
  robot_tf_data_ = std::make_unique<RosRobotTFData>();

  //Tick Periodically and very rarely (should not change but update to be sure)
  tickPeriodically();
}

void RobotTFRosBridge::tick() {
  if (ros::ok()) {
    std::string robot_name = get_isaac_robot_name();
    std::string ros_robot_name = get_ros_robot_name();
    std::vector<std::string> components = get_isaac_robot_components();
    double tick_time = getTickTime();
    double node_clock_timestamp = node()->clock()->timestamp();
    ros::Time ros_tick_time = ros::Time::now();
    for (uint i=0; i < components.size(); i++)
    {
      if (get_print_debug()){
        std::cout << robot_name << ", " << components[i] << std::endl;
        std::cout << tick_time << std::endl;
        std::cout << node_clock_timestamp << std::endl;
      }

      // get the pose tree transform for the robot to the component
      std::optional<Pose3d> robotTcomp = node()->pose().tryGet(robot_name, components[i], tick_time);

      
      if (robotTcomp){
        tf::Transform transform;
        if (get_print_debug()){
          // print the pose tree transform info (temp)
          print_pose(*robotTcomp);
        }
        // create transform and broadcast it
        auto translation = robotTcomp->translation;
        auto quaternion = robotTcomp->rotation.quaternion();
        transform.setOrigin(tf::Vector3(translation[0], translation[1], translation[2]));
        transform.setRotation(tf::Quaternion(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()));
        robot_tf_data_->tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros_tick_time, ros_robot_name, ros_robot_name + "_" + components[i]));
      }
    }
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void RobotTFRosBridge::stop() {
  robot_tf_data_ = nullptr;
}

RobotTFRosBridge::~RobotTFRosBridge() {
}

RobotTFRosBridge::RobotTFRosBridge() {
}

}  // namespace rosbridge
}  // namespace isaac
#include "ros_bridge_gt_pose.hpp"

#include "ros/ros.h"

// package to enable use of Odometry ROS messages
#include "nav_msgs/Odometry.h"

// package to enable broadcasting of transforms in ROS
#include "tf/transform_broadcaster.h"

int get_sign(float number){
  // not good code, for 0 should return zero but for my needs should work fine
  if (number >= 0){
    return 1;
  } else {
    return -1;
  }
}

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
struct GTPoseBridge::GTPoseBridgeData {
  ros::NodeHandle node;
  tf::TransformBroadcaster tf_broadcaster;
};

void GTPoseBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // Initialise data struct and define what topic node is publishing on for ROS
  gt_pose_data_ = std::make_unique<GTPoseBridgeData>();

  //Tick Periodically and very rapidly
  tickPeriodically();
}

void GTPoseBridge::tick() {
  std::vector<std::string> robot_names = get_isaac_robot_names();
  std::vector<std::string> odom_frame_names = get_isaac_odom_frame_names();

  if (robot_names.size() != odom_frame_names.size()){
    LOG_ERROR("Must have exactly one odom_frame_name for each robot_name");
    return;
  }

  if (ros::ok()) {
    double tick_time = getTickTime();
    // double node_clock_timestamp = node()->clock()->timestamp();
    ros::Time ros_tick_time = ros::Time::now();
    // std::cout << robot_names.size() << std::endl;
    for (uint i=0; i < robot_names.size(); i++)
    {
      std::string odom_frame_name = odom_frame_names[i];
      std::string robot_name = robot_names[i];
      // std::cout << robot_name << std::endl;
      // std::cout << odom_frame_name << std::endl;
      // std::cout << tick_time << std::endl;
      // std::cout << node_clock_timestamp << std::endl;

      // get the pose tree transform for the world to the robot
      std::optional<Pose3d> worldTrobot = node()->pose().tryGet("world", robot_name, tick_time);

      // // get the pose tree transform for the world to the robot's odometry frame
      std::optional<Pose3d> worldTrobot_odom = node()->pose().tryGet("world", odom_frame_name, tick_time);

      // // get the pose tree transform for the world to the robot initial pose
      // Note, we assume this has naming convention <robot_name>_init
      std::optional<Pose3d> worldTrobot_init = node()->pose().tryGet("world", robot_name + "_init", tick_time);

      // TODO check if there is good compact way to do this
      // ROBOT POSE WITHIN WORLD
      if (worldTrobot){
        tf::Transform transform;
        // print the pose tree transform info (temp)
        // print_pose(*worldTrobot);
        // create transform and broadcast it
        auto translation = worldTrobot->translation;
        auto quaternion = worldTrobot->rotation.quaternion();
        transform.setOrigin(tf::Vector3(translation[0], translation[1], translation[2]));
        transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
        
        gt_pose_data_->tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros_tick_time, "world", "gt_" + robot_name));
      }

      // ROBOT INITIAL POSITION WITHIN WORLD
      if (worldTrobot_init){
        tf::Transform transform;
        // print the pose tree transform info (temp)
        // print_pose(*robotTcomp);
        // create transform and broadcast it
        auto translation = worldTrobot_init->translation;
        auto quaternion = worldTrobot_init->rotation.quaternion();
        transform.setOrigin(tf::Vector3(translation[0], translation[1], translation[2]));
        transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
        
        gt_pose_data_->tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros_tick_time, "world", "gt_" + robot_name + "_odom"));
      }

      // ROBOT ASSUMED ODOM POSITION WITHIN WORLD
      if (worldTrobot_odom){
        tf::Transform transform;
        // print the pose tree transform info (temp)
        // print_pose(*robotTcomp);
        // create transform and broadcast it
        auto translation = worldTrobot_odom->translation;
        auto quaternion = worldTrobot_odom->rotation.quaternion();
        transform.setOrigin(tf::Vector3(translation[0], translation[1], translation[2]));
        transform.setRotation(tf::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()));
        
        gt_pose_data_->tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros_tick_time, "world", robot_name + "_odom"));
      }
    }
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void GTPoseBridge::stop() {
  gt_pose_data_ = nullptr;
}

GTPoseBridge::~GTPoseBridge() {
}

GTPoseBridge::GTPoseBridge() {
}

}  // namespace rosbridge
}  // namespace isaac
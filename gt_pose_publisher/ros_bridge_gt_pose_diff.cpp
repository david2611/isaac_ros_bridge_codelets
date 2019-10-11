#include "ros_bridge_gt_pose_diff.hpp"


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

geometry_msgs::TransformStamped pose3d_to_tf(Pose3d pose, std::string parent_frame, std::string child_frame, ros::Time time)
{
  // Convert to ROS TF and publish to ROS
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = time;
  tf.header.frame_id = parent_frame;
  tf.child_frame_id = child_frame;

  tf.transform.translation.x = pose.translation[0];
  tf.transform.translation.y = pose.translation[1];
  tf.transform.translation.z = pose.translation[2];

  // Grab the quaternion from the Pose3d (must be quicker/easier way to do things)
  auto quaternion = pose.rotation.quaternion();
  tf.transform.rotation.x = quaternion.x();
  tf.transform.rotation.y = quaternion.y();
  tf.transform.rotation.z = quaternion.z();
  tf.transform.rotation.w = quaternion.w();

  return tf;
}

// Internal struct for holding the ROS node handle and the publisher/ImageTransport components
struct GTPoseDiffTFRosBridge::RosTFData {
  ros::NodeHandle node;
  tf::TransformBroadcaster tf_broadcaster;
};

void GTPoseDiffTFRosBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // Initialise data struct and define what topic node is publishing on for ROS
  tf_data_ = std::make_unique<RosTFData>();

  //Tick Periodically and very rarely (should not change but update to be sure)
  tickPeriodically();
}

void GTPoseDiffTFRosBridge::tick() {
  if (ros::ok()) {
      auto tick_time = node()->clock()->timestamp();
       ros::Time ros_time_now = ros::Time::now();
      // Get the pose of the robot w.r.t. odom (estimated position w.r.t. start)
      std::optional<Pose3d> est_odomTrobot = node()->pose().tryGet(get_isaac_odom_name(), get_isaac_robot_name(), tick_time);

      // Get the pose of the gt_robot w.r.t. gt_robot_init (actual position w.r.t. start)
      std::optional<Pose3d> gt_odomTrobot = node()->pose().tryGet(get_isaac_gt_odom_name(), get_isaac_gt_robot_name(), tick_time);

      // Get the pose of the gt_robot_init w.r.t. the gt_world (actual position of start in world coordinates)
      std::optional<Pose3d> gt_worldTodom = node()->pose().tryGet(get_isaac_gt_world_name(), get_isaac_gt_odom_name(), tick_time);

      // Only proceed if both poses have been read successfully
      if (est_odomTrobot && gt_odomTrobot && gt_worldTodom){
        // Calculate the pose of the gt_robot with respect to the estimated robot
        // This should be the difference between the estimated position and the actual position
        const Pose3d rhs = *est_odomTrobot;
        const Pose3d lhs = *gt_odomTrobot;
        Pose3d gt_robotTrobot = lhs*rhs.inverse();

        if (get_print_debug()){
          // Sanity check printing
          std::cout << "---------------------------------------------------------------------" << std::endl;
          print_pose(*est_odomTrobot);
          std::cout << "---------------------------------------------------------------------" << std::endl;
          std::cout << "=====================================================================" << std::endl;
          print_pose(*gt_odomTrobot);
          std::cout << "=====================================================================" << std::endl;
          std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
          print_pose(gt_robotTrobot);
          std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        }

        // Convert diff to ROS TF and publish to ROS
        geometry_msgs::TransformStamped ros_robot_diff_tf = pose3d_to_tf(gt_robotTrobot, get_ros_parent_frame(), get_ros_child_frame(), ros_time_now);
        // broadcast the transform message
        tf_data_->tf_broadcaster.sendTransform(ros_robot_diff_tf);

        // Convert world ROS TF and publish to ROS
        geometry_msgs::TransformStamped ros_world_tf = pose3d_to_tf(*gt_worldTodom, get_ros_world_frame(), get_ros_parent_frame(), ros_time_now);
        // broadcast the transform message
        tf_data_->tf_broadcaster.sendTransform(ros_world_tf);
      }

  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void GTPoseDiffTFRosBridge::stop() {
  tf_data_ = nullptr;
}

GTPoseDiffTFRosBridge::~GTPoseDiffTFRosBridge() {
}

GTPoseDiffTFRosBridge::GTPoseDiffTFRosBridge() {
}

}  // namespace rosbridge
}  // namespace isaac
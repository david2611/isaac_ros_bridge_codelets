#include "ros_bridge_odom.hpp"

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

struct OdometryRosBridge::RosOdometryData{
  ros::NodeHandle node;
  ros::Publisher pub;
  tf::TransformBroadcaster ros_odom_tf_broadcaster;
};

void OdometryRosBridge::start() {
  // Setup the ROS node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  odom_data_ = std::make_unique<RosOdometryData>();
  odom_data_->pub = odom_data_->node.advertise<nav_msgs::Odometry>( 
      get_ros_publisher_channel_name(), get_ros_publisher_queue_size());

  // TODO Update below to tick on new odometry message if possible
  // tickPeriodically();
  tickOnMessage(rx_odometry());
}

void OdometryRosBridge::tick() {
  // std::optional<Pose3d> temp_robot_pose = node()->pose().tryGet("world", "map", node()->clock()->timestamp());
  // if (temp_robot_pose){
  //   std::cout << "--------------------------------------------------------------" << std::endl;
  //   print_pose(*temp_robot_pose);
  //   std::cout << "==============================================================-" << std::endl;
  // }
  
  ros::Time ros_time_now = ros::Time::now();
  // Read what is within the odometry message so as to understand it
  auto proto_reader = rx_odometry().getProto();
  // odometry transform LOOK UP EASIER WAY TO READ TRANSFORMS
  Pose2dProto::Reader odom_tf = proto_reader.getOdomTRobot();
  Vector2dProto::Reader odom_tf_translation = odom_tf.getTranslation();
  SO2dProto::Reader odom_tf_rotation = odom_tf.getRotation();
  
  // Speed
  Vector2dProto::Reader speed = proto_reader.getSpeed();
  float angular_speed = proto_reader.getAngularSpeed();
  Vector2dProto::Reader acceleration = proto_reader.getAcceleration();
  
  // frame names
  std::string odometry_frame = proto_reader.getOdometryFrame().cStr();
  std::string robot_frame = proto_reader.getRobotFrame().cStr();

  // print contents of odometry message to understand it (debugging only)
  if (get_print_isaac_odometry()){
    // print Transform
  std::cout << "odomTRobot: Translation - [ " << odom_tf_translation.getX();
  std::cout << ", " << odom_tf_translation.getY() << " ]\n";
  // NOTE need to adjust rotation to get degrees, this is just the raw stored data
  std::cout << "odomTRobot: Rotation - [ " << odom_tf_rotation.getQ().getX();
  std::cout << ", " << odom_tf_rotation.getQ().getY() << " ]\n";

  // print speed
  std::cout << "Speed: - [ " << speed.getX() << ", " << speed.getY() << " ]\n";
  std::cout << "Angular Speed: " << angular_speed << "\n";
  std::cout << "Acceleration: - [ " << acceleration.getX() << ", " << acceleration.getY() << " ]\n";

  // Print frames
  std::cout << "odometry_frame: " << odometry_frame << "\n";
  std::cout << "robot_frame: " << robot_frame << "\n";
  }
  
  // get the rotation angle (theta) from Isaac as inverse cosine of X element of SO2dProto
  // Could also use inverse sine of Y element of S02dProto (see SO2dProto documentation)
  float rotation_angle = std::abs(std::acos(odom_tf_rotation.getQ().getX())) * get_sign(std::asin(odom_tf_rotation.getQ().getY()));
  if (get_print_isaac_odometry()){
  std::cout << "rotation_angle: " << (360*rotation_angle)/(2*3.14159262) << "\n";
  }
  
  // Calculate Quaternion message needed for odometry in ROS
  geometry_msgs::Quaternion ros_odom_quat = tf::createQuaternionMsgFromYaw(rotation_angle);

  // create transform message defining where reference frame exists
  geometry_msgs::TransformStamped ros_odom_tf;
  ros_odom_tf.header.stamp = ros_time_now;
  // TODO UPDATE to make these ISAAC_PARAMs
  ros_odom_tf.header.frame_id = get_ros_header_frame();
  ros_odom_tf.child_frame_id = get_ros_child_frame();

  ros_odom_tf.transform.translation.x = odom_tf_translation.getX();
  ros_odom_tf.transform.translation.y = odom_tf_translation.getY();
  ros_odom_tf.transform.translation.z = 0.0;
  ros_odom_tf.transform.rotation = ros_odom_quat;
  // broadcast the transform message
  odom_data_->ros_odom_tf_broadcaster.sendTransform(ros_odom_tf);

  // create odometry message
  nav_msgs::Odometry ros_odom;
  // create header
  ros_odom.header.stamp = ros_time_now;
  ros_odom.header.frame_id = get_ros_header_frame();
 
  // set position
  ros_odom.pose.pose.position.x = odom_tf_translation.getX();
  ros_odom.pose.pose.position.y = odom_tf_translation.getY();
  ros_odom.pose.pose.position.z = 0.0;
  ros_odom.pose.pose.orientation = ros_odom_quat; 

  // set velocity
  ros_odom.child_frame_id = get_ros_child_frame();
  ros_odom.twist.twist.linear.x = speed.getX();
  ros_odom.twist.twist.linear.y = speed.getY();
  ros_odom.twist.twist.angular.z = angular_speed;

  // set unknowns
  // TODO UPDATE and check that this is done appropriately.
  // Based entirely off information on prediction noise from isaac.navigation.DifferentialBaseOdometry
  // prediction std dev is 0.05 for pos_x and pos_y, 0.35 for heading, 0.05 for speed, and 1.0 for angular speed
  ros_odom.pose.covariance[0] = 0.05 * 0.05;
  ros_odom.pose.covariance[7] = 0.05 * 0.05;
  ros_odom.pose.covariance[35] = 0.35 * 0.35;

  ros_odom.twist.covariance[0] = 0.05 * 0.05;
  ros_odom.twist.covariance[35] = 1.0;




  odom_data_->pub.publish(ros_odom);

}

void OdometryRosBridge::stop() {
  odom_data_->pub.shutdown();
  odom_data_ = nullptr;
}

OdometryRosBridge::~OdometryRosBridge() {

}

OdometryRosBridge::OdometryRosBridge() {

}


}// namespace rosbridge
} // namespace isaac
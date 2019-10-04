#include "isaac_gt_pose.hpp"
#include "messages/math.hpp"
namespace isaac {


void print_pose(Pose3d pose){ 
  auto translation = pose.translation;
  // auto rpy = pose.rotation.eulerAnglesRPY();
  // auto quaternion = pose.rotation.quaternion();
  // print translation
  std::cout << "Translation: {X: " << translation[0] << ", Y: " << translation[1] ;
  std::cout << ", Z: " << translation[2] << "}\n";
  // print rotation roll pitch yaw
  // std::cout << "RPY: {Roll: " << rpy[0] << ", Pitch: " << rpy[1] << ", Yaw: ";
  // std::cout << rpy[2] << "}\n";
  // print rotation quaternion
  // std::cout << "Quaternion: {W: " << quaternion.w() << ", X: " << quaternion.x() << ", Y: ";
  // std::cout << quaternion.y() << ", Z: " << quaternion.z() << "}\n";
}
// void print_pose(Pose3dProto::Reader pose){ 
//   auto translation = pose.getTranslation();
//   // auto rpy = pose.getRotation().GetEulerAnglesRPY();
//   // auto quaternion = pose.getRotation().getQuaternion();
//   // print translation
//   std::cout << "Translation: {X: " << translation.getX() << ", Y: " << translation.getY() ;
//   std::cout << ", Z: " << translation.getZ() << "}\n";
//   // print rotation roll pitch yaw
//   // std::cout << "RPY: {Roll: " << rpy[0] << ", Pitch: " << rpy[1] << ", Yaw: ";
//   // std::cout << rpy[2] << "}\n";
//   // // print rotation quaternion
//   // std::cout << "Quaternion: {W: " << quaternion.w() << ", X: " << quaternion.x() << ", Y: ";
//   // std::cout << quaternion.y() << ", Z: " << quaternion.z() << "}\n";
// }


void GTPosePublisher::start() {
  // tickPeriodically();
  tickOnMessage(rx_gt_poses());
}

void GTPosePublisher::tick(){
  auto tick_time = node()->clock()->timestamp();
  auto proto_reader = rx_gt_poses().getProto();

  auto names_list_reader = proto_reader.getNames();
  auto bodies_list_reader = proto_reader.getBodies();

  // Find the rigid body proto for the robot we are looking for 
  // (defined by get_rigid_body_robot_name)
  for (uint i = 0; i < names_list_reader.size(); i++){
    if (names_list_reader[i] == get_rigid_body_robot_name()){
      RigidBody3Proto::Reader robot_rigid_body_reader = bodies_list_reader[i];
      // std::cout << get_rigid_body_robot_name() << std::endl;
      // std::cout << "----------------------------------------------------------" << std::endl;
      // print_pose(FromProto(robot_rigid_body_reader.getRefTBody()));
      // std::cout << "----------------------------------------------------------" << std::endl;
      Pose3d robot_pose = FromProto(robot_rigid_body_reader.getRefTBody());
      node()->pose().set(get_gt_world_frame_name(), get_isaac_gt_robot_name(), robot_pose, tick_time);

      // std::cout << "GT Robot_init T Robot" << std::endl;
      // std::cout << "==========================================================" << std::endl;
      // print_pose(node()->pose().get("gt_robot_init", get_isaac_gt_robot_name(), node()->clock()->timestamp()));
      // std::cout << "==========================================================" << std::endl;
    }
  }
}

void GTPosePublisher::stop() {
}

GTPosePublisher::~GTPosePublisher() {

}

GTPosePublisher::GTPosePublisher() {

}

} // namespace isaac
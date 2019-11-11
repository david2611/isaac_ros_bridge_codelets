#include "isaac_gt_pose.hpp"
// Below needed to have access to FromProto function?
#include "messages/math.hpp"
namespace isaac {


void GTPosePublisher::start() {
  tickOnMessage(rx_gt_poses());
}

void GTPosePublisher::tick(){
  auto tick_time = node()->clock()->timestamp();
  auto proto_reader = rx_gt_poses().getProto();

  auto names_list_reader = proto_reader.getNames();
  auto bodies_list_reader = proto_reader.getBodies();

  // Find the rigid body proto for the object we are looking for 
  // (defined by get_rigid_body_object_name)
  for (uint i = 0; i < names_list_reader.size(); i++){
    if (names_list_reader[i] == get_rigid_body_object_name()){
      
      // Extract the pose from the RigidBody3Proto List
      RigidBody3Proto::Reader robot_rigid_body_reader = bodies_list_reader[i];
      Pose3d object_pose = FromProto(robot_rigid_body_reader.getRefTBody());
      
      // Set pose of object in main IsaacPoseTree using names provided
      node()->pose().set(get_gt_world_frame_name(), get_isaac_gt_object_name(), object_pose, tick_time);
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
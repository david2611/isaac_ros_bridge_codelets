#include "rigid_group_reader.hpp"
namespace isaac {

void print_pose(Pose3dProto::Reader pose){ 
  auto translation = pose.getTranslation();
  // auto rpy = pose.getRotation().GetEulerAnglesRPY();
  // auto quaternion = pose.getRotation().getQuaternion();
  // print translation
  std::cout << "Translation: {X: " << translation.getX() << ", Y: " << translation.getY() ;
  std::cout << ", Z: " << translation.getZ() << "}\n";
  // print rotation roll pitch yaw
  // std::cout << "RPY: {Roll: " << rpy[0] << ", Pitch: " << rpy[1] << ", Yaw: ";
  // std::cout << rpy[2] << "}\n";
  // // print rotation quaternion
  // std::cout << "Quaternion: {W: " << quaternion.w() << ", X: " << quaternion.x() << ", Y: ";
  // std::cout << quaternion.y() << ", Z: " << quaternion.z() << "}\n";
}


void RigidGroupReader::start() {
  
  // tickPeriodically();
  tickOnMessage(rx_gt_poses());
}

void RigidGroupReader::tick(){
  auto proto_reader = rx_gt_poses().getProto();

  auto names_list_reader = proto_reader.getNames();
  auto bodies_list_reader = proto_reader.getBodies();
  std::cout << "Reference Frame: " << proto_reader.getReferenceFrame().cStr() << std::endl;
  std::cout << "Rigid Bodies Names: " << std::endl;
  for (std::string text : names_list_reader)
  {
    std::cout << text << std::endl;
  }
  std::cout << "Rigid Body Poses:" << std::endl;
  for (RigidBody3Proto::Reader rigid_body_reader: bodies_list_reader)
  {
    std::cout << "----------------------------------------------------------" << std::endl;
    print_pose(rigid_body_reader.getRefTBody());

  }
  // std::cout << names_list_proto[0] << std::endl;
  // std::cout << proto_reader.getCellSize() << std::endl;
  // double posx = proto_reader.getPositionX();
  // double posy = proto_reader.getPositionY();
  
  // std::cout << "Position DiffBase: X: " << posx << ", Y: " << posy << std::endl;
  // std::cout << proto_reader.getHeading() << proto_rader.getLinearSpeed() << std::endl;
}

void RigidGroupReader::stop() {
}

RigidGroupReader::~RigidGroupReader() {

}

RigidGroupReader::RigidGroupReader() {

}

} // namespace isaac
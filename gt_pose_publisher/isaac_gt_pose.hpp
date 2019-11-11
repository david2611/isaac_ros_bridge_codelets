#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace isaac {

// This is a codelet for allowing you to define and publish GT Poses for objects as 
// defined from Unreal but in Isaac Coordinates
// Ideally you would have this as a component for any robot you created in the simulator
class GTPosePublisher : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  GTPosePublisher();
  virtual ~GTPosePublisher();

  void start() override;
  void tick() override;
  void stop() override;

  // Full RigidBody3GroupProto that  houses all poses for all rigid bodies in the environment
  ISAAC_PROTO_RX(RigidBody3GroupProto, gt_poses);

  // Name to give gt object pose in Isaac PoseTree
  ISAAC_PARAM(std::string, isaac_gt_object_name, "gt_robot");

  // Name of the object in the RigidBody3GroupProto of the system
  ISAAC_PARAM(std::string, rigid_body_object_name);

  // Name of the gt world coordinate frame in the pose tree
  ISAAC_PARAM(std::string, gt_world_frame_name, "gt_world");

};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::GTPosePublisher);
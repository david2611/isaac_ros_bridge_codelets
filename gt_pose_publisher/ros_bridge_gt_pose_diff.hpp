#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace isaac {
namespace rosbridge {

// This codelet represents a basic bridge to ROS for publishing images.
// Every time an image is available it will convert it to a CV Mat object and use cv_bridge
// to publish the Image message to ROS
class GTPoseDiffTFRosBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  GTPoseDiffTFRosBridge();
  virtual ~GTPoseDiffTFRosBridge();

  void start() override;
  void tick() override;
  void stop() override;

  // Name of robot in Isaac PoseTree
  ISAAC_PARAM(std::string, isaac_robot_name, "robot");

  // Name of odometry frame in Isaac PoseTree (where the robot thinks it started from)
  ISAAC_PARAM(std::string, isaac_odom_name, "odom");

  // Name of ground-truth robot location in Isaac PoseTree (ground-truth defined by Unreal Rigid Body)
  ISAAC_PARAM(std::string, isaac_gt_robot_name, "gt_robot");

  // Name of ground-truth odometry frame in the Isaac PoseTree 
  // (ground-truth robot initialization position defined in bridge_config and matched in gt_robot_pose_initializer)
  ISAAC_PARAM(std::string, isaac_gt_odom_name, "gt_robot_init");

  // Name of ground-truth world frame in the Isaac PoseTree 
  ISAAC_PARAM(std::string, isaac_gt_world_name, "gt_world");

  // Name of ROS parent frame for the published diff transform
  ISAAC_PARAM(std::string, ros_parent_frame, "map");

  // Name of ROS world frame for the published diff transform
  ISAAC_PARAM(std::string, ros_world_frame, "world");

  // Name of ROS child frame for the published diff transform
  ISAAC_PARAM(std::string, ros_child_frame, "odom");

  // Boolean flag for printing outputs
  ISAAC_PARAM(bool, print_debug, false);

 private:
  // Hide the ROS implementation details
  struct RosTFData;
  std::unique_ptr<RosTFData> tf_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::GTPoseDiffTFRosBridge);
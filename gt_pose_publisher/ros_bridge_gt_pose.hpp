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
class GTPoseBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  GTPoseBridge();
  virtual ~GTPoseBridge();

  void start() override;
  void tick() override;
  void stop() override;

  // NOTE Currently assume structure contains:
  // 1. world -> <robot_name>_init (gt_robot_odom)
  // 2. world -> <robot_name> (gt_robot)
  // 3. world -> <odom_frame_name> (robot_odom as calculated by robot?)

  // Name of all robots we are providing ground-truth for (isaac_names)
  ISAAC_PARAM(std::vector<std::string>, isaac_robot_names, {});

  // Name of all odometry frames calculated all robots for (isaac_names)
  ISAAC_PARAM(std::vector<std::string>, isaac_odom_frame_names, {});

 private:
  // Hide the ROS implementation details
  struct GTPoseBridgeData;
  std::unique_ptr<GTPoseBridgeData> gt_pose_data_;

};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::GTPoseBridge);
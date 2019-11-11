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
class RobotTFRosBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  RobotTFRosBridge();
  virtual ~RobotTFRosBridge();

  void start() override;
  void tick() override;
  void stop() override;

  // Name of robot in Isaac PoseTree
  ISAAC_PARAM(std::string, isaac_robot_name, "robot");

  // Name of robot as it will appear in ROS
  ISAAC_PARAM(std::string, ros_robot_name, "robot");

  // Name of all components (sensors) on the robot with available poses
  ISAAC_PARAM(std::vector<std::string>, isaac_robot_components, {});

  // Flag dictating if debugging printing should happen
  ISAAC_PARAM(bool, print_debug, false);


 private:
  // Hide the ROS implementation details
  struct RosRobotTFData;
  std::unique_ptr<RosRobotTFData> robot_tf_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::RobotTFRosBridge);
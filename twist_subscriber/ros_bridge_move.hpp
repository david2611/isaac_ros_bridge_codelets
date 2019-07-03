#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace isaac {
namespace rosbridge {

// This codelet represents a basic bridge to ROS for communicating navigation events.
// It receives geometry_msg/Twist messages from ROS and controls the robot
class MovementRosBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  MovementRosBridge();
  virtual ~MovementRosBridge();

  void start() override;
  void tick() override;

  // Isaac proto output with velocity message
  ISAAC_PROTO_TX(StateProto, base_cmd);

  // ROS subscriber queue depth
  ISAAC_PARAM(int, subscriber_queue_size, 1000);

  // ROS subscriber channel. Used to receive messagse from ROS
  ISAAC_PARAM(std::string, subscriber_channel_name, "/cmd_vel");

private:

  // Hide the ROS implementation details
  struct RosMoveData;
  std::unique_ptr<RosMoveData> move_data_;


};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::MovementRosBridge);
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
class OdometryRosBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  OdometryRosBridge();
  virtual ~OdometryRosBridge();

  void start() override;
  void tick() override;
  void stop() override;

  //Incoming message channel on which we receive colour images
  ISAAC_PROTO_RX(Odometry2Proto, odometry)

  // ROS publisher queue depth
  ISAAC_PARAM(int, ros_publisher_queue_size, 1000);

  // ROS publisher channel. Used to broadcast messages to ROS
  ISAAC_PARAM(std::string, ros_publisher_channel_name, "isaac_odometry");

 private:
  // Hide the ROS implementation details
  struct RosOdometryData;
  std::unique_ptr<RosOdometryData> odom_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::OdometryRosBridge);
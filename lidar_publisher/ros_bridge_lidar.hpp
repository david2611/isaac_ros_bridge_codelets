#pragma once

#include <memory>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace isaac {
namespace rosbridge{

class LidarRosBridge : public isaac::alice::Codelet {
 public:
  LidarRosBridge() {}
  virtual ~LidarRosBridge() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PARAM(std::string, lidar_ros_topic_name, "/scan_laser");
  ISAAC_PARAM(std::string, lidar_ros_frame_name, "lidar");

  ISAAC_PROTO_RX(FlatscanProto, lidar_scan);

 private:
  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace rosbridge
}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::LidarRosBridge);

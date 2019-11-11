#pragma once

#include <memory>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace isaac {
namespace rosbridge{

class LidarPublisher : public isaac::alice::Codelet {
 public:
  LidarPublisher() {}
  virtual ~LidarPublisher() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PARAM(std::string, lidar_channel_name, "/scan_laser");
  ISAAC_PARAM(std::string, lidar_frame_name, "lidar");

  ISAAC_PROTO_RX(FlatscanProto, lidar_scan);

 private:
  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace rosbridge
}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::LidarPublisher);

#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "engine/core/image/image.hpp"
#include "messages/camera.hpp"
#include "messages/messages.hpp"

namespace isaac {
namespace rosbridge {
namespace opencv{

// This codelet represents a basic bridge to ROS for publishing images.
// Every time an image is available it will convert it to a CV Mat object and use cv_bridge
// to publish the Image message to ROS
class DepthImageRosBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  DepthImageRosBridge();
  virtual ~DepthImageRosBridge();

  void start() override;
  void tick() override;
  void stop() override;

  //Incoming message channel on which we receive colour images
  ISAAC_PROTO_RX(DepthCameraProto, depth_image)

  // ROS publisher queue depth
  ISAAC_PARAM(int, ros_publisher_queue_size, 1000);

  // ROS publisher channel. Used to broadcast messages to ROS
  ISAAC_PARAM(std::string, ros_publisher_channel_name, "isaac_depth_image");

  // Name of frame used in ROS (format <robot_name>_<camera_name>)
  ISAAC_PARAM(std::string, ros_frame_name, "robot_left_camera")

 private:
  // Hide the ROS implementation details
  struct RosDepthImageData;
  std::unique_ptr<RosDepthImageData> depth_img_data_;
};

}  // namespace opencv
}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::opencv::DepthImageRosBridge);
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

// This codelet represents a basic bridge to ROS for publishing depth images.
// Every time a depth image is available it will convert it to a CV Mat object and 
// use cv_bridge to publish the message to ROS
class SegmentImageRosBridge : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  SegmentImageRosBridge();
  virtual ~SegmentImageRosBridge();

  void start() override;
  void tick() override;
  void stop() override;

  //Incoming message channel on which we receive colour images
  ISAAC_PROTO_RX(SegmentationCameraProto, segment_image);

  // ROS publisher queue depth
  ISAAC_PARAM(int, ros_publisher_queue_size, 1000);

  // ROS publisher channel. Used to broadcast messages to ROS
  ISAAC_PARAM(std::string, ros_publish_topic, "isaac_segment");



  // Name of frame used in ROS (format <robot_name>_<camera_name>)
  ISAAC_PARAM(std::string, ros_frame_name, "robot_left_camera");

 private:
  // Hide the ROS implementation details
  struct RosSegmentImageData;
  std::unique_ptr<RosSegmentImageData> segment_img_data_;
};

}  // namespace opencv
}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::opencv::SegmentImageRosBridge);
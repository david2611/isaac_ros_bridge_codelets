#include "ros_bridge_depth_img.hpp"

//#include "engine/core/assert.hpp"

#include "ros/ros.h"

// needed for the image transport specifically
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

namespace isaac {
namespace rosbridge {
namespace opencv {

// Internal struct for holding the ROS node handle and the publisher/ImageTransport components
struct DepthImageRosBridge::RosDepthImageData {
  ros::NodeHandle node;
  image_transport::Publisher pub;
  image_transport::ImageTransport it = image_transport::ImageTransport(this->node);
};

void DepthImageRosBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // Initialise data struct and define what topic node is publishing on for ROS
  depth_img_data_ = std::make_unique<RosDepthImageData>();
  depth_img_data_->pub = depth_img_data_->it.advertise(
      get_ros_publisher_channel_name(), get_ros_publisher_queue_size());

  //Tick on message since only publishing. If subscribing need to tick periodically
  tickOnMessage(rx_depth_image());
}

void DepthImageRosBridge::tick() {
  if (ros::ok()) {
    
    ros::Time ros_time_now = ros::Time::now();

    // parse the received messages
    auto proto1 = rx_depth_image().getProto();
    ImageConstView1f depth_image;
    bool ok = FromProto(proto1.getDepthImage(), rx_depth_image().buffers(), depth_image);
    
    if (ok) {
        // convert the Image Proto into a CV Mat
        const size_t rows = depth_image.rows();
        const size_t cols = depth_image.cols();
        cv::Mat cv_depth_image = cv::Mat(rows, cols, CV_32FC1,
                        const_cast<void*>(static_cast<const void*>(depth_image.data().pointer())));
        
        sensor_msgs::ImagePtr ros_depth_img_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", cv_depth_image).toImageMsg();
        ros_img_msg->header.stamp = ros_time_now;
        ros_img_msg->header.frame_id = get_ros_frame_name();
        depth_img_data_->pub.publish(ros_depth_img_msg);

    }
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void DepthImageRosBridge::stop() {
  depth_img_data_->pub.shutdown();
  depth_img_data_ = nullptr;
}

DepthImageRosBridge::~DepthImageRosBridge() {
}

DepthImageRosBridge::DepthImageRosBridge() {
}

}  // namespace opencv
}  // namespace rosbridge
}  // namespace isaac
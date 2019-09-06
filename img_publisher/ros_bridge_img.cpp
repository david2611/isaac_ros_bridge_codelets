#include "ros_bridge_img.hpp"

//#include "engine/core/assert.hpp"

#include "ros/ros.h"

// needed for the image transport specifically
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

// not sure why these aren't needed (opencv in particular)
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc.hpp"
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>

namespace isaac {
namespace rosbridge {
namespace opencv {

// Internal struct for holding the ROS node handle and the publisher/ImageTransport components
struct ImageRosBridge::RosImageData {
  ros::NodeHandle node;
  image_transport::Publisher pub;
  image_transport::ImageTransport it = image_transport::ImageTransport(this->node);
};

void ImageRosBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // Initialise data struct and define what topic node is publishing on for ROS
  img_data_ = std::make_unique<RosImageData>();
  img_data_->pub = img_data_->it.advertise(
      get_ros_publisher_channel_name(), get_ros_publisher_queue_size());

  //Tick on message since only publishing. If subscribing need to tick periodically
  tickOnMessage(rx_rgbCapture_ros());
}

void ImageRosBridge::tick() {
  if (ros::ok()) {
    ros::Time ros_time_now = ros::Time::now();
    // parse the received messages
    auto proto1 = rx_rgbCapture_ros().getProto();
    ImageConstView3ub rgb_image;
    bool ok = FromProto(proto1.getImage(), rx_rgbCapture_ros().buffers(), rgb_image);
    
    if (ok) {
        // convert the Image Proto into a CV Mat
        const size_t rows = rgb_image.rows();
        const size_t cols = rgb_image.cols();
        cv::Mat image = cv::Mat(rows, cols, CV_8UC3,
                        const_cast<void*>(static_cast<const void*>(rgb_image.data().pointer())));
        cv::cvtColor(image, image, CV_RGB2BGR);
        
        // convert cv Mat to  an image pointer message with cv_bridge
        // need to update the header?
        sensor_msgs::ImagePtr ros_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        ros_img_msg->header.stamp = ros_time_now;
        ros_img_msg->header.frame_id = get_ros_frame_name();
        img_data_->pub.publish(ros_img_msg);

    }
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void ImageRosBridge::stop() {
  img_data_->pub.shutdown();
  img_data_ = nullptr;
}

ImageRosBridge::~ImageRosBridge() {
}

ImageRosBridge::ImageRosBridge() {
}

}  // namespace opencv
}  // namespace rosbridge
}  // namespace isaac
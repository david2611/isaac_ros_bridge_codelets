#include "ros_bridge_segment_img.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

// needed for the image transport specifically
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <isaac_ros_msgs/isaac_segment_img.h>
#include <sensor_msgs/image_encodings.h>

namespace isaac {
namespace rosbridge {
namespace opencv {

template<typename type>
std::vector<type> unique(cv::Mat in) {
    assert(in.channels() == 1 && "This implementation is only for single-channel images");
    auto begin = in.begin<type>(), end = in.end<type>();
    auto last = std::unique(begin, end);    // remove adjacent duplicates to reduce size
    std::sort(begin, last);                 // sort remaining elements
    last = std::unique(begin, last);        // remove duplicates
    return std::vector<type>(begin, last);
}

void print_min_max(cv::Mat in){
  double minVal; 
  double maxVal; 
  cv::Point minLoc; 
  cv::Point maxLoc;
  cv::minMaxLoc(in, &minVal, &maxVal, &minLoc, &maxLoc );

  std::cout << "min val: " << minVal << std::endl;
  std::cout << "max val: " << maxVal << std::endl;
}

template<typename type>
void print_vector(std::vector<type> input){
  std::cout << "[";
        for (auto i = input.begin(); i != input.end(); ++i)
std::cout << (uint16_t) *i << ' ';
        std::cout << "]" << std::endl;
}


// Internal struct for holding the ROS node handle and the publisher/ImageTransport components
struct SegmentImageRosBridge::RosSegmentImageData {
  ros::NodeHandle node;
  ros::Publisher pub_isaac_segment;
};

void SegmentImageRosBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // Initialise data struct and define what topic node is publishing on for ROS
  segment_img_data_ = std::make_unique<RosSegmentImageData>();
  segment_img_data_->pub_isaac_segment = segment_img_data_->node.advertise<isaac_ros_msgs::isaac_segment_img>(get_ros_publish_topic(), get_ros_publisher_queue_size());

  //Tick on message since only publishing. If subscribing need to tick periodically
  tickOnMessage(rx_segment_image());
}

void SegmentImageRosBridge::tick() {
  if (ros::ok()) {
    
    ros::Time ros_time_now = ros::Time::now();

    // parse the received messages
    auto segment_proto = rx_segment_image().getProto();
    ImageConstView1ub label_image;
    bool ok = FromProto(segment_proto.getLabelImage(), rx_segment_image().buffers(), label_image);
    // NOTE I am not certain if a different buffer is needed for instance and label images
    ImageConstView1ui16 instance_image;
    ok = ok && FromProto(segment_proto.getInstanceImage(), rx_segment_image().buffers(), instance_image);
    
    sensor_msgs::ImagePtr ros_label_img_msg;
    sensor_msgs::ImagePtr ros_instance_img_msg;
    if (ok) {
        // convert the Image Proto into a CV Mat
        const size_t rows = label_image.rows();
        const size_t cols = label_image.cols();
        // Use 8UC1 for label image format and 16UC1 for instance
        cv::Mat cv_label_image = cv::Mat(rows, cols, CV_8UC1,
                        const_cast<void*>(static_cast<const void*>(label_image.data().pointer())));
        cv::Mat cv_instance_image = cv::Mat(rows, cols, CV_16UC1,
                        const_cast<void*>(static_cast<const void*>(instance_image.data().pointer())));
        
        ros_label_img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", cv_label_image).toImageMsg();
        ros_instance_img_msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_instance_image).toImageMsg();

        // Update header information before publishing
        ros_label_img_msg->header.stamp = ros_time_now;
        ros_label_img_msg->header.frame_id = get_ros_frame_name();
        ros_instance_img_msg->header.stamp = ros_time_now;
        ros_instance_img_msg->header.frame_id = get_ros_frame_name();

        isaac_ros_msgs::isaac_segment_img segment_msg;
        segment_msg.class_segment_img = *ros_label_img_msg;
        segment_msg.instance_segment_img = *ros_instance_img_msg;
        for (auto label_proto : segment_proto.getLabels()) {
          segment_msg.class_names.push_back(label_proto.getName().cStr());
          segment_msg.class_ids.push_back(label_proto.getIndex() + 1);
        }
        segment_img_data_->pub_isaac_segment.publish(segment_msg);
    }

    

    
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void SegmentImageRosBridge::stop() {
  segment_img_data_->pub_isaac_segment.shutdown();
  segment_img_data_ = nullptr;
}

SegmentImageRosBridge::~SegmentImageRosBridge() {
}

SegmentImageRosBridge::SegmentImageRosBridge() {
}

}  // namespace opencv
}  // namespace rosbridge
}  // namespace isaac
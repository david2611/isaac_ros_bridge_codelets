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
        cv::Mat cv_instance_image_original = cv::Mat(rows, cols, CV_16UC1,
                        const_cast<void*>(static_cast<const void*>(instance_image.data().pointer())));
        auto instance_ids_original = unique<uint16_t>(cv_instance_image_original.clone());
        auto class_ids = unique<uint8_t>(cv_label_image.clone());
        
        
        // TODO create new instance image to put new values into
        cv::Mat cv_instance_image_final = cv::Mat::zeros(cv_instance_image_original.rows, 
                                                     cv_instance_image_original.cols,
                                                     CV_16UC1);
        // Create vector of counts for each class id
        std::vector<int> class_inst_counts(class_ids.size(), 0);

        // Go through all instance ids in the original
        for (auto inst_id = instance_ids_original.begin(); inst_id != instance_ids_original.end(); ++inst_id)
        {
          if (*inst_id > 0)
          {
            // Get the mask of the current instance id
            cv::Mat inst_mask;
            cv::inRange(cv_instance_image_original, *inst_id, *inst_id, inst_mask);

            // Perform binary operation with class label image to find classes within this instance id
            cv::Mat masked_label_image;
            cv_label_image.copyTo(masked_label_image, inst_mask);
            // Go through all class ids in the masked image
            auto masked_class_ids = unique<uint8_t>(masked_label_image.clone());

            // print_min_max(masked_label_image);
            // print_min_max(cv_label_image);
            // print_vector(unique<uint8_t>(masked_label_image.clone()));
            // print_vector(unique<uint8_t>(cv_label_image.clone()));

            for (auto cls_id = masked_class_ids.begin(); cls_id != masked_class_ids.end(); ++cls_id)
            {
              if ((uint16_t) *cls_id > 0){
                // Increase count of instances for given class by 1
                int cls_count_idx = std::distance(class_ids.begin(), 
                                                  std::find(class_ids.begin(), 
                                                            class_ids.end(), 
                                                            *cls_id));

                class_inst_counts[cls_count_idx]++;

                // define a new instance id based on the class id and instance id
                // New id is instance id + 100 times class id
                // NOTE this only works for fewer than 1000 instances and 65 classes
                uint16_t new_inst_id = (uint16_t) *cls_id * 1000 + class_inst_counts[cls_count_idx];
                // std::cout << "New Instance ID: " << new_inst_id <<std::endl;

                // Define the mask for the instance
                cv::Mat cls_inst_mask;
                cv::inRange(masked_label_image, *cls_id, *cls_id, cls_inst_mask);

                // Set values in the final instance image to the new value
                cv_instance_image_final.setTo(new_inst_id, cls_inst_mask);
              }
            }
            
          }
        }

        // print_vector(unique<uint16_t>(cv_instance_image_final.clone()));
        // print_min_max(cv_instance_image_original);
        // std::cout << "M = " << std::endl << " "  << cv_instance_image << std::endl << std::endl;
        // print_vector(instance_ids_original);
        
        ros_label_img_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC1", cv_label_image).toImageMsg();
        ros_instance_img_msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_instance_image_final).toImageMsg();

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
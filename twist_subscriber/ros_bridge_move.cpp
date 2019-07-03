#include "ros_bridge_move.hpp"

//#include "engine/core/assert.hpp"

#include "ros/ros.h"
// below needed for using Twist message in ROS
#include "geometry_msgs/Twist.h"
// below needed to use the callback queue
#include "ros/callback_queue.h"

// below needed to use DifferentialBaseControl struct
#include "messages/state/differential_base.hpp"
// below needed to gain access to ToProto function
#include "engine/gems/state/io.hpp"

namespace isaac {
namespace rosbridge {

namespace {
// Callback functor to avoid having a callback function in the bridge class
class CallbackFunctor {
 public:
  explicit CallbackFunctor(MovementRosBridge* bridge) {
    bridge_ = bridge;
  }
  
  CallbackFunctor(const CallbackFunctor&) = default;
  ~CallbackFunctor() = default;

  void operator()(const geometry_msgs::Twist::ConstPtr& msg) {
    // currently assume linear velocity is in linear x and 
    // angular velocity is in angular z based on observations
    messages::DifferentialBaseControl command;
    command.linear_speed() = 2*msg->linear.x;
    command.angular_speed() = msg->angular.z;

    // uncomment below to visualise the speeds as a debug check.
    // std::cout << "Linear: " << command.linear_speed() << "\tAngular: " << command.angular_speed() <<"\n";
    
    // Use the DifferentialBaseControl to create proto to be sent to robot
    ToProto(command, bridge_->tx_base_cmd().initProto());
    // Publish Proto message
    bridge_->tx_base_cmd().publish();
  }
  private:
    MovementRosBridge* bridge_;
};
}  // namespace

// Internal struct for holding the ROS node handle and the publisher and subscriber channels
// Note the callback queue. To avoid spinning other codelets it is necessary to generate
// a separate callback queue per codelet.
struct MovementRosBridge::RosMoveData {
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::CallbackQueue callbackQueue;
};

void MovementRosBridge::start() {
  // Setup the ros node and communication channels
  // Make sure to disable the Sigint handler and if any args are needed
  // add them to the args string.
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  // Setup the move_data_ node that subscribes to a ROS topic
  move_data_ = std::make_unique<RosMoveData>();
  move_data_->node.setCallbackQueue(&(move_data_->callbackQueue));
  move_data_->sub = move_data_->node.subscribe<geometry_msgs::Twist>(
      get_subscriber_channel_name(), get_subscriber_queue_size(), CallbackFunctor(this));

  //Tick on message since only publishing. If subscribing need to tick periodically
  tickPeriodically();
}

void MovementRosBridge::tick() {
  if (ros::ok()) {
    // Pump the navigation data from ROS to Isaac through the codelet
    move_data_->callbackQueue.callAvailable();
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }

}

MovementRosBridge::~MovementRosBridge() {
}

MovementRosBridge::MovementRosBridge() {
}

}  // namespace rosbridge
}  // namespace isaac
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace isaac {

// This codelet represents a basic bridge to ROS for publishing images.
// Every time an image is available it will convert it to a CV Mat object and use cv_bridge
// to publish the Image message to ROS
class DiffBaseReader : public alice::Codelet {
 public:
  // Explicitly declare constructors and destructors
  // to get around forward declaration of RosNavigationData
  DiffBaseReader();
  virtual ~DiffBaseReader();

  void start() override;
  void tick() override;
  void stop() override;

  //Incoming message channel on which we receive colour images
  ISAAC_PROTO_RX(DifferentialState, base_state)
  
};
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::DiffBaseReader);
#include "base_state_reader.hpp"
namespace isaac {



void DiffBaseReader::start() {
  
  // tickPeriodically();
  tickOnMessage(rx_base_state());
}

void DiffBaseReader::tick(){
  auto proto_reader = rx_base_state().getProto();
  // std::cout << proto_reader.getCellSize() << std::endl;
  // double posx = proto_reader.getPositionX();
  // double posy = proto_reader.getPositionY();
  
  // std::cout << "Position DiffBase: X: " << posx << ", Y: " << posy << std::endl;
  std::cout << proto_reader.getHeading() << proto_reader.getSpeedX() << proto_reader.getTimestamp() << std::endl;
}

void DiffBaseReader::stop() {
}

DiffBaseReader::~DiffBaseReader() {

}

DiffBaseReader::DiffBaseReader() {

}

} // namespace isaac
#include "mock_serial_interface/mock_driver.hpp"
#include <iostream>
#include <algorithm>

namespace mock_serial_interface
{

MockDriver::MockDriver()
{
}

MockDriver::~MockDriver()
{
  close();
}

bool MockDriver::open(const std::string & port_name)
{
  port_name_ = port_name;
  connected_ = true;
  std::cout << "[MockDriver] Opening virtual serial port: " << port_name_ << std::endl;
  return true;
}

void MockDriver::close()
{
  if (connected_) {
    std::cout << "[MockDriver] Closing virtual serial port: " << port_name_ << std::endl;
    connected_ = false;
  }
}

bool MockDriver::is_connected() const
{
  return connected_;
}

void MockDriver::write_packet(const std::vector<uint8_t> & packet)
{
  if (!connected_) return;

  // Basic packet validation
  if (packet.size() < 4) {
    // Invalid packet
    return;
  }

  // Simulate processing time/jitter? 
  // For now, just parse directly.
  parse_packet(packet);
}

std::vector<uint8_t> MockDriver::read_packet()
{
  if (!connected_) return {};

  // In a real reading scenario, we'd buffer bytes.
  // Here we just demand-generate a packet representing current state.
  return create_feedback_packet();
}

void MockDriver::update(double dt)
{
  if (!connected_) return;

  // Simple P-controller-like simulation or max velocity ramp
  double diff = target_position_ - current_position_;
  double move_step = max_velocity_ * dt;

  if (std::abs(diff) < move_step) {
    current_position_ = target_position_;
    current_velocity_ = 0.0; // Reached target
  } else {
    if (diff > 0) {
      current_position_ += move_step;
      current_velocity_ = max_velocity_;
    } else {
      current_position_ -= move_step;
      current_velocity_ = -max_velocity_;
    }
  }
}

void MockDriver::parse_packet(const std::vector<uint8_t> & packet)
{
  // Protocol: [0xAA, CMD, VAL_H, VAL_L]
  // 0xAA = Header
  // CMD = 0x01 (Set Position)
  // VAL = int16_t mapped to position * 100
  
  if (packet[0] != 0xAA) return;
  
  uint8_t cmd = packet[1];
  if (cmd == 0x01) { // Set Target
    int16_t val = (static_cast<int16_t>(packet[2]) << 8) | packet[3];
    target_position_ = static_cast<double>(val) / 100.0;
    // std::cout << "[MockDriver] Received Target: " << target_position_ << std::endl;
  }
}

std::vector<uint8_t> MockDriver::create_feedback_packet()
{
  // Protocol: [0xBB, STATUS, POS_H, POS_L]
  std::vector<uint8_t> packet;
  packet.push_back(0xBB); // Header
  packet.push_back(0x00); // System Status OK

  int16_t pos_int = static_cast<int16_t>(current_position_ * 100.0);
  packet.push_back((pos_int >> 8) & 0xFF);
  packet.push_back(pos_int & 0xFF);
  
  return packet;
}

} // namespace mock_serial_interface

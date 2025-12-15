#ifndef MOCK_SERIAL_INTERFACE__MOCK_DRIVER_HPP_
#define MOCK_SERIAL_INTERFACE__MOCK_DRIVER_HPP_

#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <chrono>

namespace mock_serial_interface
{

class MockDriver
{
public:
  MockDriver();
  ~MockDriver();

  bool open(const std::string & port_name);
  void close();
  bool is_connected() const;

  // Simulate writing a command packet
  // Format: [HEADER, CMD, VAL_H, VAL_L, CHECKSUM]
  void write_packet(const std::vector<uint8_t> & packet);

  // Simulate reading a feedback packet
  // Returns empty if no data available
  std::vector<uint8_t> read_packet();

  // Update simulation state (call this periodically)
  void update(double dt);

  double get_position() const { return current_position_; }
  double get_velocity() const { return current_velocity_; }

private:
  bool connected_ = false;
  std::string port_name_;
  
  // Simulated hardware state
  double current_position_ = 0.0;
  double current_velocity_ = 0.0;
  double target_position_ = 0.0;
  
  // Motor parameters
  const double max_velocity_ = 2.0; // rad/s (approx)

  void parse_packet(const std::vector<uint8_t> & packet);
  std::vector<uint8_t> create_feedback_packet();
};

} // namespace mock_serial_interface

#endif // MOCK_SERIAL_INTERFACE__MOCK_DRIVER_HPP_

#include "mock_serial_interface/mock_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mock_serial_interface
{

hardware_interface::CallbackReturn MockHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage for the interfaces
  hw_states_position_ = 0.0;
  hw_states_velocity_ = 0.0;
  hw_commands_position_ = 0.0;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Mock Serial Interface has only one joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MockHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MockHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MockHardwareInterface"),
        "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string port_name = info_.hardware_parameters["port_name"];
  if (driver_.open(port_name)) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn MockHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset command to current state to prevent jumps
  hw_commands_position_ = hw_states_position_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MockHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MockHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Simulate driver update loop
  driver_.update(period.seconds());

  // In a real generic serial driver, we might just pump bytes here 
  // or rely on a separate thread. For the mock, we stimulate it here.
  
  auto packet = driver_.read_packet();
  if (!packet.empty() && packet[0] == 0xBB) {
      // Decode feedback
      int16_t val = (static_cast<int16_t>(packet[2]) << 8) | packet[3];
      hw_states_position_ = static_cast<double>(val) / 100.0;
      hw_states_velocity_ = driver_.get_velocity(); // Cheating a bit, getting direct velocity
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MockHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Encode command
  // [0xAA, 0x01, VAL_H, VAL_L]
  std::vector<uint8_t> packet;
  packet.push_back(0xAA);
  packet.push_back(0x01);
  
  int16_t cmd_int = static_cast<int16_t>(hw_commands_position_ * 100.0);
  packet.push_back((cmd_int >> 8) & 0xFF);
  packet.push_back(cmd_int & 0xFF);

  driver_.write_packet(packet);

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MockHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MockHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_));
  }

  return command_interfaces;
}

} // namespace mock_serial_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mock_serial_interface::MockHardwareInterface,
  hardware_interface::SystemInterface)

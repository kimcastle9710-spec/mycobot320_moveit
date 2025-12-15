#ifndef MYCOBOT_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_
#define MYCOBOT_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mycobot/mycobot.hpp"

namespace mycobot
{
class MyCobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyCobotHardwareInterface)

  // [Humble Fix] CallbackReturn 타입 정의
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Lifecycle Node Interfaces
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // [누락되었던 부분 추가!] Export Interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read / Write
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string serial_port_;
  int baud_rate_;
  std::unique_ptr<MyCobot> mycobot_;
  
  // Store the command and state for the joints
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace mycobot

#endif  // MYCOBOT_HARDWARE_INTERFACE__HARDWARE_INTERFACE_HPP_

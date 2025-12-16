#include "mycobot_hardware_interface/hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <array>
#include <algorithm> // for copy_n

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mycobot
{

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  // hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_.resize(info_.joints.size(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    // [수정] 생성자에서 포트와 보드레이트를 넣으면 내부적으로 연결 시도함
    // open()이나 is_open() 함수가 없으므로 호출하지 않음
    mycobot_ = std::make_unique<MyCobot>(serial_port_, baud_rate_);
    
    // 연결 안정화 대기
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyCobotHardwareInterface"), "Exception during connection: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("MyCobotHardwareInterface"), "MyCobot Initialized on %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyCobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyCobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  read(rclcpp::Time(), rclcpp::Duration(0,0));
  for (uint i = 0; i < hw_states_.size(); i++) {
      if (!std::isnan(hw_states_[i])) {
          hw_commands_[i] = hw_states_[i];
      } else {
          hw_commands_[i] = 0.0;
      }
  }
  RCLCPP_INFO(rclcpp::get_logger("MyCobotHardwareInterface"), "System Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyCobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyCobotHardwareInterface"), "System successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// hardware_interface::return_type MyCobotHardwareInterface::read(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
// {
//   if (!mycobot_) return hardware_interface::return_type::ERROR;

//   // [수정] get_radians() 사용 (리턴 타입: fp::Result<std::array<double, 6>>)
//   auto result = mycobot_->get_radians(); 
  
//   // fp::Result 처리 (유효한 값이 있는지 확인)
//   if (result) { // 혹은 result.has_value() 
//       auto angles = *result; // 값 추출
//       if (angles.size() == hw_states_.size()) {
//           for (size_t i = 0; i < angles.size(); ++i) {
//               hw_states_[i] = angles[i]; 
//           }
//       }
//   } else {
//       // 읽기 실패 시 에러 로그를 띄우거나 무시 (너무 자주 뜨면 주석 처리)
//       // RCLCPP_WARN(rclcpp::get_logger("MyCobotHardwareInterface"), "Failed to read angles");
//   }
  
//   return hardware_interface::return_type::OK;
// }

hardware_interface::return_type MyCobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!mycobot_) return hardware_interface::return_type::ERROR;

  auto result = mycobot_->get_radians();
  
  if (result) { 
      // 데이터가 있으면 업데이트
      auto angles = *result; 
      if (angles.size() == hw_states_.size()) {
          for (size_t i = 0; i < angles.size(); ++i) {
              hw_states_[i] = angles[i]; 
          }
      }
  } else {
      // [수정] 데이터 읽기 실패 시 경고 로그 출력 (NaN 에러 대신 로그 확인용)
      static int log_counter = 0;
      if (log_counter++ % 100 == 0) { 
          RCLCPP_WARN(rclcpp::get_logger("MyCobotHardwareInterface"), "로봇에서 각도를 읽을 수 없습니다! USB 연결을 확인하세요.");
      }
      // 값을 업데이트하지 않고 기존 값(또는 0.0) 유지 -> TF 에러 방지
  }
  
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type MyCobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!mycobot_) return hardware_interface::return_type::ERROR;

  if (std::any_of(hw_commands_.begin(), hw_commands_.end(), [](double val){ return std::isnan(val); })) {
      return hardware_interface::return_type::OK;
  }

  // [수정] send_radians() 사용을 위해 std::vector -> std::array 변환
  std::array<double, 6> commands_array;
  if (hw_commands_.size() >= 6) {
      std::copy_n(hw_commands_.begin(), 6, commands_array.begin());
      
      // send_radians(각도, 속도) 호출. 속도는 0~100 사이. 80 정도로 설정.
      mycobot_->send_radians(commands_array, 80);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mycobot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mycobot::MyCobotHardwareInterface, hardware_interface::SystemInterface)

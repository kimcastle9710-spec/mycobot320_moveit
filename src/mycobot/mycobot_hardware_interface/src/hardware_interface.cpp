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


hardware_interface::return_type MyCobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!mycobot_) return hardware_interface::return_type::ERROR;

  // 1. 하드웨어에서 6개 관절 값 가져오기 (무조건 J1, J2, J3, J4, J5, J6 순서)
  auto result = mycobot_->get_radians();

  if (result) {
      auto angles = *result; 

      // 2. ROS가 요구하는 관절 순서(info_.joints)를 하나씩 확인하며 "제 짝"을 찾아줌
      for (size_t i = 0; i < info_.joints.size(); ++i) {
          std::string name = info_.joints[i].name;
          double val = 0.0;
          bool is_arm_joint = false;

          // [핵심] 이름표를 보고 angles의 몇 번째 값을 가져올지 매핑
          if (name == "joint2_to_joint1") {       // URDF: Joint 1
              val = angles[0]; is_arm_joint = true;
          } 
          else if (name == "joint3_to_joint2") {  // URDF: Joint 2
              val = angles[1]; is_arm_joint = true;
          }
          else if (name == "joint4_to_joint3") {  // URDF: Joint 3
              val = angles[2]; is_arm_joint = true;
          }
          else if (name == "joint5_to_joint4") {  // URDF: Joint 4
              val = angles[3]; is_arm_joint = true;
          }
          else if (name == "joint6_to_joint5") {  // URDF: Joint 5
              val = angles[4]; is_arm_joint = true;
          }
          else if (name == "joint6output_to_joint6") { // URDF: Joint 6 (Mimic)
              val = angles[5]; is_arm_joint = true;
          }
          else if (name == "gripper_controller") { // 그리퍼
              hw_states_[i] = hw_commands_[i];     // Loopback
              continue; 
          }

          // 3. 값 정규화 (-3.14 ~ +3.14) 후 대입
          if (is_arm_joint) {
              while (val > M_PI) val -= 2.0 * M_PI;
              while (val < -M_PI) val += 2.0 * M_PI;
              hw_states_[i] = val;
          }
      }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!mycobot_) return hardware_interface::return_type::ERROR;

  // NaN(유효하지 않은 값) 체크
  if (std::any_of(hw_commands_.begin(), hw_commands_.end(), [](double val){ return std::isnan(val); })) {
      return hardware_interface::return_type::OK;
  }

  // ---------------------------------------------------------
  // 1. 로봇 팔 제어 (과부하 방지 필터 추가)
  // ---------------------------------------------------------
  std::array<double, 6> commands_array;
  if (hw_commands_.size() >= 6) {
      std::copy_n(hw_commands_.begin(), 6, commands_array.begin());

      // [핵심] 이전 명령과 비교해서 변화가 미미하면 전송 스킵!
      static std::array<double, 6> last_commands = {0};
      double total_diff = 0.0;
      
      for(size_t i=0; i<6; ++i) {
          total_diff += std::abs(commands_array[i] - last_commands[i]);
      }

      // 변화량 합계가 0.02 라디안(약 1도) 미만이면 명령 안 보냄 (떨림 방지)
      // *주의: 움직임이 너무 끊기면 이 값을 0.01로 줄이세요.
      if (total_diff > 0.02) { 
          mycobot_->send_radians(commands_array, 80); // 속도 80%
          last_commands = commands_array; // 마지막 명령 업데이트
      }
  }

  // ---------------------------------------------------------
  // 2. 그리퍼 제어 (중복 전송 방지 포함됨)
  // ---------------------------------------------------------
  for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (info_.joints[i].name == "gripper_controller") {
          double cmd = hw_commands_[i];
          static int last_gripper_state = -1; 

          // 닫기 (-0.5 미만)
          if (cmd < -0.5 && last_gripper_state != 1) {
              mycobot_->send(set_gripper_state(1, 50)); 
              last_gripper_state = 1; 
          } 
          // 열기 (-0.2 초과)
          else if (cmd > -0.2 && last_gripper_state != 0) {
              mycobot_->send(set_gripper_state(0, 50));
              last_gripper_state = 0; 
          }
      }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mycobot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mycobot::MyCobotHardwareInterface, hardware_interface::SystemInterface)

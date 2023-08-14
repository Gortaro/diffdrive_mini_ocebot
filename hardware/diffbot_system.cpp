// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_mini_ocebot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>

namespace diffdrive_mini_ocebot
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.left_wheel_pin = std::stoi(info_.hardware_parameters["left_wheel_pin"]);
  cfg_.right_wheel_pin = std::stoi(info_.hardware_parameters["right_wheel_pin"]);
  cfg_.left_direction_pin = std::stoi(info_.hardware_parameters["left_direction_pin"]);
  cfg_.right_direction_pin = std::stoi(info_.hardware_parameters["right_direction_pin"]);
  cfg_.left_enc_pin = std::stoi(info_.hardware_parameters["left_encoder_pin"]);
  cfg_.right_enc_pin = std::stoi(info_.hardware_parameters["right_encoder_pin"]);
  cfg_.enc_counts_per_rev = std::stoul(info_.hardware_parameters["enc_counts_per_rev"]);
  
  wheel_left_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_right_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_left_.name, hardware_interface::HW_IF_POSITION, &wheel_left_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_right_.name, hardware_interface::HW_IF_POSITION, &wheel_right_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_left_.name, hardware_interface::HW_IF_VELOCITY, &wheel_left_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_right_.name, hardware_interface::HW_IF_VELOCITY, &wheel_right_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  gpio_controller_.setup(cfg_.left_enc_pin, cfg_.right_enc_pin, cfg_.left_wheel_pin, cfg_.right_wheel_pin, cfg_.left_direction_pin, cfg_.right_direction_pin);
  gpio_controller_.register_encoders(wheel_left_.enc, wheel_right_.enc);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Terminating connection to daemon... please wait...");

  gpio_controller_.cleanup();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double delta_seconds = period.seconds();

  float pos_prev = wheel_left_.pos;
  wheel_left_.pos = wheel_left_.calc_enc_angle();
  wheel_left_.vel = (wheel_left_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_right_.pos;
  wheel_right_.pos = wheel_right_.calc_enc_angle();
  wheel_right_.vel = (wheel_right_.pos - pos_prev) / delta_seconds;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_mini_ocebot ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  int motor_l_counts_per_loop = wheel_left_.cmd * 20;
  int motor_r_counts_per_loop = wheel_right_.cmd * 20;

  gpio_controller_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_mini_ocebot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_mini_ocebot::DiffBotSystemHardware, hardware_interface::SystemInterface)

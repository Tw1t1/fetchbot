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

#include "diffdrive/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive
{
hardware_interface::CallbackReturn DiffDriveSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = (info_.hardware_parameters["left_wheel_name"]);
  cfg_.right_wheel_name = (info_.hardware_parameters["right_wheel_name"]);
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.ena_pin = std::stoi(info_.hardware_parameters["ena_pin"]);
  cfg_.in1_pin = std::stoi(info_.hardware_parameters["in1_pin"]);
  cfg_.in2_pin = std::stoi(info_.hardware_parameters["in2_pin"]);
  cfg_.enb_pin = std::stoi(info_.hardware_parameters["enb_pin"]);
  cfg_.in3_pin = std::stoi(info_.hardware_parameters["in3_pin"]);
  cfg_.in4_pin = std::stoi(info_.hardware_parameters["in4_pin"]);

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  motor_l_.initialize(cfg_.ena_pin, cfg_.in1_pin, cfg_.in2_pin);
  motor_r_.initialize(cfg_.enb_pin, cfg_.in3_pin, cfg_.in4_pin);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Activating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // wheel_l_.pos = wheel_l_.pos + period.seconds() * wheel_l_.vel;
  // RCLCPP_INFO(
  //     rclcpp::get_logger("DiffDriveSystemHardware"),
  //     "Got position state %.5f and velocity state %.5f for '%s'!", wheel_l_.pos,
  //     wheel_l_.vel, wheel_l_.name.c_str());

  // wheel_r_.pos = wheel_r_.pos + period.seconds() * wheel_r_.vel;
  // RCLCPP_INFO(
  //     rclcpp::get_logger("DiffDriveSystemHardware"),
  //     "Got position state %.5f and velocity state %.5f for '%s'!", wheel_r_.pos,
  //     wheel_r_.vel, wheel_r_.name.c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive ::DiffDriveSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Writing...");

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveSystemHardware"), "Got command %.5f for '%s'!", wheel_l_.cmd,
    wheel_l_.name.c_str());

  // wheel_l_.vel = wheel_l_.cmd;

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveSystemHardware"), "Got command %.5f for '%s'!", wheel_r_.cmd,
    wheel_r_.name.c_str());

  // wheel_r_.vel = wheel_r_.cmd;

  motor_l_.setVelocity(wheel_l_.cmd);
  motor_r_.setVelocity(wheel_r_.cmd);

  // RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive::DiffDriveSystemHardware, hardware_interface::SystemInterface)

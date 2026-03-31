// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
constexpr const char * kGoalPositionItem = "Goal_Position";
constexpr const char * kGoalVelocityItem = "Goal_Velocity";
constexpr const char * kMovingSpeedItem = "Moving_Speed";
constexpr const char * kPresentPositionItem = "Present_Position";
constexpr const char * kPresentVelocityItem = "Present_Velocity";
constexpr const char * kPresentSpeedItem = "Present_Speed";
constexpr const char * kPresentCurrentItem = "Present_Current";
constexpr const char * kPresentLoadItem = "Present_Load";
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity", "Profile_Acceleration", "Position_P_Gain", "Position_I_Gain",
  "Position_D_Gain",  "Velocity_P_Gain",      "Velocity_I_Gain",
};

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);
  mimic_joint_ids_.resize(info_.joints.size(), 0);
  mimic_joint_multiplier_.resize(info_.joints.size(), 0.0);
  joint_gearing_.resize(info.joints.size(), 0.0);
  position_mode_.resize(info.joints.size(), "");
  joint_goal_current_.resize(info.joints.size(), 0.0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;

    auto it = info_.joints[i].parameters.find("mimic");
    if (it != info_.joints[i].parameters.end()) {
      mimic_joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("mimic"));
      mimic_joint_multiplier_[i] = std::stod(info_.joints[i].parameters.at("multiplier"));
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger(kDynamixelHardware),
        "Joint " << info_.joints[i].name << " is a mimic of joint " << mimic_joint_ids_[i]
                 << " with multiplier " << mimic_joint_multiplier_[i]);
    } else {
      // set to itself to indicate that it is not a mimic joint
      mimic_joint_ids_[i] = joint_ids_[i];
    }

    it = info_.joints[i].parameters.find("gearing");

    if (it != info_.joints[i].parameters.end()) {
      joint_gearing_[i] = std::stod(info_.joints[i].parameters.at("gearing"));
      RCLCPP_INFO(
        rclcpp::get_logger(kDynamixelHardware), "Motor %d gearing: %f", i + 1, joint_gearing_[i]);
    } else {
      joint_gearing_[i] = 1.0;
      RCLCPP_INFO(
        rclcpp::get_logger(kDynamixelHardware), "Motor %d gearing: %f DEFAULT VALUE", i + 1,
        joint_gearing_[i]);
    }

    it = info_.joints[i].parameters.find("position_mode");

    if (it != info_.joints[i].parameters.end()) {
      std::string value = info_.joints[i].parameters.at("position_mode");

      position_mode_[i] = value;

      RCLCPP_INFO(
        rclcpp::get_logger(kDynamixelHardware), "Motor %d position mode: %s", i + 1,
        position_mode_[i].c_str());
    } else {
      position_mode_[i] = false;
      RCLCPP_INFO(
        rclcpp::get_logger(kDynamixelHardware),
        "Motor %d use_extended_position_mode: %s DEFAULT VALUE", i + 1, position_mode_[i].c_str());
    }

    it = info_.joints[i].parameters.find("goal_current");

    if (it != info_.joints[i].parameters.end()) {
      joint_goal_current_[i] = std::stod(info_.joints[i].parameters.at("goal_current"));
      RCLCPP_INFO(
        rclcpp::get_logger(kDynamixelHardware), "Motor %d goal_current: %d", i + 1,
        joint_goal_current_[i]);
    } else {
      joint_goal_current_[i] = 1.0;  //set to 1 mA
      RCLCPP_INFO(
        rclcpp::get_logger(kDynamixelHardware), "Motor %d goal_current: %d DEFAULT VALUE", i + 1,
        joint_goal_current_[i]);
    }
  }
  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

  if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint16_t model_number = 0;
    if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }
  }

  enable_torque(false);
  set_control_mode(ControlMode::Position, true);
  set_joint_params();
  enable_torque(true);

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
  if (goal_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * goal_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
  if (goal_velocity == nullptr) {
    goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
  }
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_velocity =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
  if (present_velocity == nullptr) {
    present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
  }
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_current =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
  if (present_current == nullptr) {
    present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
  }
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  for (uint i = 0; i < info_.joints.size(); ++i) {
    const ControlItem * joint_goal_position =
      dynamixel_workbench_.getItemInfo(joint_ids_[i], kGoalPositionItem);
    if (joint_goal_position == nullptr) {
      return CallbackReturn::ERROR;
    }

    const ControlItem * joint_goal_velocity =
      dynamixel_workbench_.getItemInfo(joint_ids_[i], kGoalVelocityItem);
    if (joint_goal_velocity == nullptr) {
      joint_goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[i], kMovingSpeedItem);
    }
    if (joint_goal_velocity == nullptr) {
      return CallbackReturn::ERROR;
    }

    const ControlItem * joint_present_position =
      dynamixel_workbench_.getItemInfo(joint_ids_[i], kPresentPositionItem);
    if (joint_present_position == nullptr) {
      return CallbackReturn::ERROR;
    }

    const ControlItem * joint_present_velocity =
      dynamixel_workbench_.getItemInfo(joint_ids_[i], kPresentVelocityItem);
    if (joint_present_velocity == nullptr) {
      joint_present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[i], kPresentSpeedItem);
    }
    if (joint_present_velocity == nullptr) {
      return CallbackReturn::ERROR;
    }

    const ControlItem * joint_present_current =
      dynamixel_workbench_.getItemInfo(joint_ids_[i], kPresentCurrentItem);
    if (joint_present_current == nullptr) {
      joint_present_current = dynamixel_workbench_.getItemInfo(joint_ids_[i], kPresentLoadItem);
    }
    if (joint_present_current == nullptr) {
      return CallbackReturn::ERROR;
    }

    // Assign to SyncWriteGroup
    bool found_write_group = false;
    for (auto & group : sync_write_groups_) {
      if (group.pos_address == joint_goal_position->address &&
          group.pos_length == joint_goal_position->data_length &&
          group.vel_address == joint_goal_velocity->address &&
          group.vel_length == joint_goal_velocity->data_length) {
        group.joint_ids.push_back(joint_ids_[i]);
        group.joint_indices.push_back(i);
        found_write_group = true;
        break;
      }
    }
    if (!found_write_group) {
      SyncWriteGroup new_group;
      new_group.pos_address = joint_goal_position->address;
      new_group.pos_length = joint_goal_position->data_length;
      new_group.vel_address = joint_goal_velocity->address;
      new_group.vel_length = joint_goal_velocity->data_length;
      new_group.joint_ids.push_back(joint_ids_[i]);
      new_group.joint_indices.push_back(i);
      
      if (!dynamixel_workbench_.addSyncWriteHandler(
            new_group.pos_address, new_group.pos_length, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return CallbackReturn::ERROR;
      }
      new_group.pos_index = sync_write_groups_.size() * 2; // dynamixel sdk groups index 

      if (!dynamixel_workbench_.addSyncWriteHandler(
            new_group.vel_address, new_group.vel_length, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return CallbackReturn::ERROR;
      }
      new_group.vel_index = sync_write_groups_.size() * 2 + 1;
      
      sync_write_groups_.push_back(new_group);
    }

    // Assign to SyncReadGroup
    bool found_read_group = false;
    for (auto & group : sync_read_groups_) {
      if (group.pos_address == joint_present_position->address &&
          group.pos_length == joint_present_position->data_length &&
          group.vel_address == joint_present_velocity->address &&
          group.vel_length == joint_present_velocity->data_length &&
          group.cur_address == joint_present_current->address &&
          group.cur_length == joint_present_current->data_length) {
        group.joint_ids.push_back(joint_ids_[i]);
        group.joint_indices.push_back(i);
        found_read_group = true;
        break;
      }
    }
    if (!found_read_group) {
      SyncReadGroup new_group;
      new_group.pos_address = joint_present_position->address;
      new_group.pos_length = joint_present_position->data_length;
      new_group.vel_address = joint_present_velocity->address;
      new_group.vel_length = joint_present_velocity->data_length;
      new_group.cur_address = joint_present_current->address;
      new_group.cur_length = joint_present_current->data_length;
      new_group.joint_ids.push_back(joint_ids_[i]);
      new_group.joint_indices.push_back(i);

      uint16_t start_address = std::min(new_group.pos_address, new_group.cur_address);
      uint16_t end_address = std::max(
        (uint16_t)(new_group.pos_address + new_group.pos_length),
        (uint16_t)(std::max(
          (uint16_t)(new_group.cur_address + new_group.cur_length),
          (uint16_t)(new_group.vel_address + new_group.vel_length)
        ))
      );
      uint16_t read_length = end_address - start_address;

      if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return CallbackReturn::ERROR;
      }
      new_group.index = sync_read_groups_.size();

      sync_read_groups_.push_back(new_group);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "start");
  for (uint i = 0; i < joints_.size(); i++) {
    if (use_dummy_ && std::isnan(joints_[i].state.position)) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }
  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "stop");
  return CallbackReturn::SUCCESS;
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  const char * log = nullptr;

  for (auto & group : sync_read_groups_) {
    std::vector<int32_t> positions(group.joint_ids.size(), 0);
    std::vector<int32_t> velocities(group.joint_ids.size(), 0);
    std::vector<int32_t> currents(group.joint_ids.size(), 0);

    if (!dynamixel_workbench_.syncRead(
          group.index, group.joint_ids.data(), group.joint_ids.size(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
          group.index, group.joint_ids.data(), group.joint_ids.size(),
          group.cur_address, group.cur_length, currents.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
          group.index, group.joint_ids.data(), group.joint_ids.size(),
          group.vel_address, group.vel_length, velocities.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
          group.index, group.joint_ids.data(), group.joint_ids.size(),
          group.pos_address, group.pos_length, positions.data(), &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    for (uint i = 0; i < group.joint_ids.size(); i++) {
      uint index = group.joint_indices[i];
      joints_[index].state.position =
        dynamixel_workbench_.convertValue2Radian(group.joint_ids[i], positions[i]) / joint_gearing_[index];
      joints_[index].state.velocity = dynamixel_workbench_.convertValue2Velocity(group.joint_ids[i], velocities[i]);
      joints_[index].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
    }
  }

  return return_type::OK;
}

return_type DynamixelHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  //printout all joint positions
  // std::ostringstream oss;
  // for (size_t i = 0; i < joints_.size(); ++i) {
  //   if (i > 0) oss << ", ";
  //   oss << "joint" << (i + 1) << ": " << joints_[i].command.position;
  // }
  // std::string result = oss.str();

  // RCLCPP_ERROR_STREAM(rclcpp::get_logger(kDynamixelHardware), result);

  //ceck if command is nan then replace with previous command
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (std::isnan(joints_[i].command.position)) {
      // RCLCPP_INFO(
      //   rclcpp::get_logger(kDynamixelHardware),
      //   "=========== DEBUG POSTOZERO AVOID motor: %ld command: %f to prevcommand: %f ===========",
      //  i + 1, joints_[i].command.position, joints_[i].prev_command.position);

      joints_[i].command.position = joints_[i].prev_command.position;
    }
  }

  if (use_dummy_) {
    for (auto & joint : joints_) {
      joint.prev_command.position = joint.command.position;
      joint.state.position = joint.command.position;
    }
    return return_type::OK;
  }

  // Velocity control
  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto j) {
        return j.command.velocity != j.prev_command.velocity;
      })) {
    set_control_mode(ControlMode::Velocity);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_velocities();
    return return_type::OK;
  }

  for (uint8_t i = 0; i < joints_.size(); i++) {
    if (mimic_joint_ids_[i] != joint_ids_[i]) {
      // get index of mimic joint
      auto it = std::find_if(joint_ids_.begin(), joint_ids_.end(), [&](uint8_t id) {
        return id == mimic_joint_ids_[i];
      });

      auto index = std::distance(joint_ids_.begin(), it);

      joints_[i].command.position = joints_[index].command.position * mimic_joint_multiplier_[i];
    }
  }

  // Position control
  if (std::any_of(joints_.cbegin(), joints_.cend(), [](auto j) {
        //RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "====== DEBUG: command.position: %f, prev_command.position: %f ======", j.command.position, j.prev_command.position);
        return j.command.position != j.prev_command.position;
      })) {
    //RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "====== DEBUG: 396 Position Control Mode ======");

    set_control_mode(ControlMode::Position);
    if (mode_changed_) {
      set_joint_params();
    }
    set_joint_positions();
    return return_type::OK;
  }

  // Effort control
  if (std::any_of(
        joints_.cbegin(), joints_.cend(), [](auto j) { return j.command.effort != 0.0; })) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Effort control is not implemented");
    return return_type::ERROR;
  }

  // If all command values are unchanged, then remain in existing control mode and set
  // corresponding command values
  switch (control_mode_) {
    case ControlMode::Velocity:
      set_joint_velocities();
      return return_type::OK;
      break;
    case ControlMode::Position:
      set_joint_positions();
      return return_type::OK;
      break;
    default:  // effort, etc
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Control mode not implemented");
      return return_type::ERROR;
      break;
  }
}

return_type DynamixelHardware::disable_invert_drive()
{
  const char * log = nullptr;

  for (uint i = 0; i < info_.joints.size(); ++i) {
    if (!dynamixel_workbench_.itemWrite(joint_ids_[i], "Drive_Mode", 0, &log)) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return return_type::ERROR;
    }
  }

  return return_type::OK;
}

return_type DynamixelHardware::enable_torque(const bool enabled)
{
  const char * log = nullptr;

  if (enabled && !torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    reset_command();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
  } else if (!enabled && torque_enabled_) {
    for (uint i = 0; i < info_.joints.size(); ++i) {
      if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
  }

  torque_enabled_ = enabled;
  return return_type::OK;
}

return_type DynamixelHardware::set_control_mode(const ControlMode & mode, const bool force_set)
{
  const char * log = nullptr;
  mode_changed_ = false;

  if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity)) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "velocity control1");
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }

    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log)) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
        return return_type::ERROR;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");
    if (control_mode_ != ControlMode::Velocity) {
      mode_changed_ = true;
      control_mode_ = ControlMode::Velocity;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  //set mode for each motor individually based on param
  if (mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position)) {
    bool torque_enabled = torque_enabled_;
    if (torque_enabled) {
      enable_torque(false);
    }
    for (uint i = 0; i < joint_ids_.size(); ++i) {
      if (position_mode_[i] == "position") {
        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware), "Motor %d  set Position Control mode", i + 1);

        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware), "=========== DEBUG 519 SET POSMODE ===========");

        if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      } else if (position_mode_[i] == "extendedPosition") {
        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware), "Motor %d  set ExtendedPosition Control mode",
          i + 1);
        if (!dynamixel_workbench_.setExtendedPositionControlMode(joint_ids_[i], &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          RCLCPP_INFO(
            rclcpp::get_logger(kDynamixelHardware),
            "Motor %d  FAILED to set ExtendedPosition Control mode", i + 1);
          return return_type::ERROR;
        }
      } else if (position_mode_[i] == "currentBasedPosition") {
        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware),
          "Motor %d  set CurrentBassedPosition Control mode", i + 1);
        if (!dynamixel_workbench_.writeRegister(
              joint_ids_[i], "Operating_Mode", 5, &log)) {  //5= CURRENT_BASED_POSITION_CONTROL_MODE
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          RCLCPP_INFO(
            rclcpp::get_logger(kDynamixelHardware), "Motor %d  FAILED to set Position Control mode",
            i + 1);
          return return_type::ERROR;
        }

        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware),
          "===== DEBUG 554 set goalCurrent %d ======", joint_goal_current_[i]);

        if (!dynamixel_workbench_.writeRegister(
              joint_ids_[i], "Goal_Current", static_cast<int32_t>(joint_goal_current_[i]),
              &log)) {  // 2.69 mA
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          RCLCPP_FATAL(
            rclcpp::get_logger(kDynamixelHardware),
            "Motor %d  FAILED to set Goal current Control mode", i + 1);
          return return_type::ERROR;
        }
      } else {
        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware), "- failed to set Mode control mode silent fail");
      }
    }
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware), "set control mode ExtendedPosition control");
    if (control_mode_ != ControlMode::Position) {
      mode_changed_ = true;
      //RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "====== DEBUG: mode changed ======");
      control_mode_ = ControlMode::Position;
    }

    if (torque_enabled) {
      enable_torque(true);
    }
    return return_type::OK;
  }

  if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
    return return_type::ERROR;
  }

  return return_type::OK;
}

return_type DynamixelHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++) {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
    joints_[i].prev_command.position = joints_[i].command.position;
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    joints_[i].prev_command.effort = joints_[i].command.effort;
  }

  return return_type::OK;
}

CallbackReturn DynamixelHardware::set_joint_positions()
{
  const char * log = nullptr;
  
  for (auto & group : sync_write_groups_) {
    std::vector<int32_t> commands(group.joint_ids.size(), 0);
    for (uint i = 0; i < group.joint_ids.size(); i++) {
      uint index = group.joint_indices[i];
      joints_[index].prev_command.position = joints_[index].command.position;
      commands[i] = dynamixel_workbench_.convertRadian2Value(
        group.joint_ids[i], static_cast<float>(joints_[index].command.position * joint_gearing_[index]));
    }
    if (!dynamixel_workbench_.syncWrite(
          group.pos_index, group.joint_ids.data(), group.joint_ids.size(), commands.data(), 1, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;

  for (auto & group : sync_write_groups_) {
    std::vector<int32_t> commands(group.joint_ids.size(), 0);
    for (uint i = 0; i < group.joint_ids.size(); i++) {
      uint index = group.joint_indices[i];
      joints_[index].prev_command.velocity = joints_[index].command.velocity;
      commands[i] = dynamixel_workbench_.convertVelocity2Value(
        group.joint_ids[i], static_cast<float>(joints_[index].command.velocity));
    }
    if (!dynamixel_workbench_.syncWrite(
          group.vel_index, group.joint_ids.data(), group.joint_ids.size(), commands.data(), 1, &log)) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_params()
{
  const char * log = nullptr;
  for (uint i = 0; i < info_.joints.size(); ++i) {
    for (auto paramName : kExtraJointParameters) {
      if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end()) {
        auto value = std::stoi(info_.joints[i].parameters.at(paramName));
        if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log)) {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
          rclcpp::get_logger(kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
      }
    }
  }
  return CallbackReturn::SUCCESS;
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)

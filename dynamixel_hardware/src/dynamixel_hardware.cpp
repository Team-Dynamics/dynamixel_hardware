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
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
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
constexpr double kSuspiciousReportedPositionRad = 1000.0;
constexpr const char * const kExtraJointParameters[] = {
  "Profile_Velocity", "Profile_Acceleration", "Position_P_Gain", "Position_I_Gain",
  "Position_D_Gain",  "Velocity_P_Gain",      "Velocity_I_Gain",
};

std::vector<uint8_t> parse_joint_id_list(const std::string & raw)
{
  std::vector<uint8_t> ids;
  std::stringstream ss(raw);
  std::string token;

  while (std::getline(ss, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
    if (token.empty()) {
      continue;
    }
    if (token == "none") {
      ids.clear();
      return ids;
    }

    char * end = nullptr;
    const long parsed = std::strtol(token.c_str(), &end, 10);
    if (*end != '\0' || parsed < 0 || parsed > 253) {
      RCLCPP_WARN(
        rclcpp::get_logger(kDynamixelHardware),
        "Ignoring invalid ID '%s' in debug_joint_ids. Expected comma-separated uint8 values.",
        token.c_str());
      continue;
    }
    ids.push_back(static_cast<uint8_t>(parsed));
  }

  return ids;
}

const ControlItem * get_item_with_fallback(
  DynamixelWorkbench & workbench, const uint8_t id, const char * primary_item,
  const char * fallback_item, const char ** selected_item_name)
{
  const ControlItem * item = workbench.getItemInfo(id, primary_item);
  if (item != nullptr) {
    if (selected_item_name != nullptr) {
      *selected_item_name = primary_item;
    }
    return item;
  }

  item = workbench.getItemInfo(id, fallback_item);
  if (item != nullptr && selected_item_name != nullptr) {
    *selected_item_name = fallback_item;
  }

  return item;
}

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
        "Joint " << info_.joints[i].name << " is a mimic of joint "
                 << static_cast<int>(mimic_joint_ids_[i])
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

  debug_joint_ids_.clear();
  auto debug_joint_ids_it = info_.hardware_parameters.find("debug_joint_ids");
  if (debug_joint_ids_it != info_.hardware_parameters.end()) {
    debug_joint_ids_ = parse_joint_id_list(debug_joint_ids_it->second);
  }

  direct_read_joint_ids_ = {7, 8};
  auto direct_read_joint_ids_it = info_.hardware_parameters.find("direct_read_joint_ids");
  if (direct_read_joint_ids_it != info_.hardware_parameters.end()) {
    direct_read_joint_ids_ = parse_joint_id_list(direct_read_joint_ids_it->second);
  }

  auto debug_every_n_reads_it = info_.hardware_parameters.find("debug_every_n_reads");
  if (debug_every_n_reads_it != info_.hardware_parameters.end()) {
    const int parsed = std::stoi(debug_every_n_reads_it->second);
    if (parsed > 0) {
      debug_every_n_reads_ = static_cast<uint32_t>(parsed);
    }
  }

  if (!debug_joint_ids_.empty()) {
    std::stringstream ids_str;
    for (size_t i = 0; i < debug_joint_ids_.size(); ++i) {
      if (i > 0) {
        ids_str << ",";
      }
      ids_str << static_cast<int>(debug_joint_ids_[i]);
    }
    RCLCPP_WARN(
      rclcpp::get_logger(kDynamixelHardware),
      "Joint probe diagnostics enabled for IDs [%s], every %u reads or on suspicious positions.",
      ids_str.str().c_str(), debug_every_n_reads_);
  }

  if (!direct_read_joint_ids_.empty()) {
    std::stringstream ids_str;
    for (size_t i = 0; i < direct_read_joint_ids_.size(); ++i) {
      if (i > 0) {
        ids_str << ",";
      }
      ids_str << static_cast<int>(direct_read_joint_ids_[i]);
    }
    RCLCPP_WARN(
      rclcpp::get_logger(kDynamixelHardware),
      "Direct itemRead fallback enabled for IDs [%s] to bypass syncRead decoding errors.",
      ids_str.str().c_str());
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
    RCLCPP_INFO(
      rclcpp::get_logger(kDynamixelHardware), "Ping success for joint '%s' (ID=%d), model=%u",
      info_.joints[i].name.c_str(), joint_ids_[i], model_number);
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

  const ControlItem * goal_velocity = get_item_with_fallback(
    dynamixel_workbench_, joint_ids_[0], kGoalVelocityItem, kMovingSpeedItem, nullptr);
  if (goal_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const ControlItem * present_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
  if (present_position == nullptr) {
    return CallbackReturn::ERROR;
  }

  const char * selected_present_velocity_name = nullptr;
  const ControlItem * present_velocity = get_item_with_fallback(
    dynamixel_workbench_, joint_ids_[0], kPresentVelocityItem, kPresentSpeedItem,
    &selected_present_velocity_name);
  if (present_velocity == nullptr) {
    return CallbackReturn::ERROR;
  }

  const char * selected_present_current_name = nullptr;
  const ControlItem * present_current = get_item_with_fallback(
    dynamixel_workbench_, joint_ids_[0], kPresentCurrentItem, kPresentLoadItem,
    &selected_present_current_name);
  if (present_current == nullptr) {
    return CallbackReturn::ERROR;
  }

  control_items_[kGoalPositionItem] = goal_position;
  control_items_[kGoalVelocityItem] = goal_velocity;
  control_items_[kPresentPositionItem] = present_position;
  control_items_[kPresentVelocityItem] = present_velocity;
  control_items_[kPresentCurrentItem] = present_current;

  RCLCPP_INFO(
    rclcpp::get_logger(kDynamixelHardware),
    "Reference control table from ID=%d: %s(addr=%u,len=%u) %s(addr=%u,len=%u) %s(addr=%u,len=%u)",
    joint_ids_[0], kPresentPositionItem, control_items_[kPresentPositionItem]->address,
    control_items_[kPresentPositionItem]->data_length, selected_present_velocity_name,
    control_items_[kPresentVelocityItem]->address, control_items_[kPresentVelocityItem]->data_length,
    selected_present_current_name, control_items_[kPresentCurrentItem]->address,
    control_items_[kPresentCurrentItem]->data_length);

  for (uint i = 0; i < info_.joints.size(); ++i) {
    const char * this_velocity_name = nullptr;
    const char * this_current_name = nullptr;
    const ControlItem * this_position =
      dynamixel_workbench_.getItemInfo(joint_ids_[i], kPresentPositionItem);
    const ControlItem * this_velocity = get_item_with_fallback(
      dynamixel_workbench_, joint_ids_[i], kPresentVelocityItem, kPresentSpeedItem,
      &this_velocity_name);
    const ControlItem * this_current = get_item_with_fallback(
      dynamixel_workbench_, joint_ids_[i], kPresentCurrentItem, kPresentLoadItem,
      &this_current_name);

    if (this_position == nullptr || this_velocity == nullptr || this_current == nullptr) {
      RCLCPP_WARN(
        rclcpp::get_logger(kDynamixelHardware),
        "Joint '%s' (ID=%d) has missing control item(s): pos=%d vel=%d current/load=%d",
        info_.joints[i].name.c_str(), joint_ids_[i], this_position != nullptr,
        this_velocity != nullptr, this_current != nullptr);
      continue;
    }

    if (
      this_position->address != control_items_[kPresentPositionItem]->address ||
      this_position->data_length != control_items_[kPresentPositionItem]->data_length ||
      this_velocity->address != control_items_[kPresentVelocityItem]->address ||
      this_velocity->data_length != control_items_[kPresentVelocityItem]->data_length ||
      this_current->address != control_items_[kPresentCurrentItem]->address ||
      this_current->data_length != control_items_[kPresentCurrentItem]->data_length) {
      RCLCPP_WARN(
        rclcpp::get_logger(kDynamixelHardware),
        "Control table mismatch for joint '%s' (ID=%d): pos(addr=%u,len=%u) %s(addr=%u,len=%u) %s(addr=%u,len=%u)",
        info_.joints[i].name.c_str(), joint_ids_[i], this_position->address,
        this_position->data_length, this_velocity_name, this_velocity->address,
        this_velocity->data_length, this_current_name, this_current->address,
        this_current->data_length);
    }
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
        &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  const uint16_t pos_start = control_items_[kPresentPositionItem]->address;
  const uint16_t vel_start = control_items_[kPresentVelocityItem]->address;
  const uint16_t cur_start = control_items_[kPresentCurrentItem]->address;
  const uint16_t pos_end = pos_start + control_items_[kPresentPositionItem]->data_length;
  const uint16_t vel_end = vel_start + control_items_[kPresentVelocityItem]->data_length;
  const uint16_t cur_end = cur_start + control_items_[kPresentCurrentItem]->data_length;

  const uint16_t start_address = std::min({pos_start, vel_start, cur_start});
  const uint16_t end_address = std::max({pos_end, vel_end, cur_end});
  const uint16_t read_length = end_address - start_address;
  if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
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

bool DynamixelHardware::is_debug_joint(const uint8_t id) const
{
  return std::find(debug_joint_ids_.begin(), debug_joint_ids_.end(), id) != debug_joint_ids_.end();
}

bool DynamixelHardware::is_direct_read_joint(const uint8_t id) const
{
  return std::find(direct_read_joint_ids_.begin(), direct_read_joint_ids_.end(), id) !=
         direct_read_joint_ids_.end();
}

return_type DynamixelHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_) {
    return return_type::OK;
  }

  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<uint8_t> sync_ids;
  std::vector<size_t> sync_joint_indices;
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  for (size_t i = 0; i < ids.size(); ++i) {
    if (!is_direct_read_joint(ids[i])) {
      sync_ids.push_back(ids[i]);
      sync_joint_indices.push_back(i);
    }
  }

  const char * log = nullptr;
  bool fallback_all_to_direct_position = false;

  if (!sync_ids.empty()) {
    std::vector<int32_t> sync_positions(sync_ids.size(), 0);
    std::vector<int32_t> sync_velocities(sync_ids.size(), 0);
    std::vector<int32_t> sync_currents(sync_ids.size(), 0);

    const bool sync_ok = dynamixel_workbench_.syncRead(
      kPresentPositionVelocityCurrentIndex, sync_ids.data(), sync_ids.size(), &log);
    if (!sync_ok) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      fallback_all_to_direct_position = true;
    } else {
      const bool curr_ok = dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, sync_ids.data(), sync_ids.size(),
        control_items_[kPresentCurrentItem]->address,
        control_items_[kPresentCurrentItem]->data_length, sync_currents.data(), &log);
      const bool vel_ok = dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, sync_ids.data(), sync_ids.size(),
        control_items_[kPresentVelocityItem]->address,
        control_items_[kPresentVelocityItem]->data_length, sync_velocities.data(), &log);
      const bool pos_ok = dynamixel_workbench_.getSyncReadData(
        kPresentPositionVelocityCurrentIndex, sync_ids.data(), sync_ids.size(),
        control_items_[kPresentPositionItem]->address,
        control_items_[kPresentPositionItem]->data_length, sync_positions.data(), &log);

      if (!curr_ok || !vel_ok || !pos_ok) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "groupSyncRead getdata failed");
        fallback_all_to_direct_position = true;
      } else {
        for (size_t i = 0; i < sync_ids.size(); ++i) {
          const size_t joint_index = sync_joint_indices[i];
          positions[joint_index] = sync_positions[i];
          velocities[joint_index] = sync_velocities[i];
          currents[joint_index] = sync_currents[i];
        }
      }
    }
  }

  ++read_cycle_count_;

  for (uint i = 0; i < ids.size(); ++i) {
    if (!is_direct_read_joint(ids[i]) && !fallback_all_to_direct_position) {
      continue;
    }

    int32_t direct_position = 0;

    const bool direct_position_ok =
      dynamixel_workbench_.itemRead(ids[i], kPresentPositionItem, &direct_position, &log);

    if (direct_position_ok) {
      positions[i] = direct_position;
    }

    if (!direct_position_ok) {
      RCLCPP_WARN(
        rclcpp::get_logger(kDynamixelHardware),
        "Direct position fallback failed for ID=%d. Last log: %s",
        ids[i],
        log != nullptr ? log : "<none>");
    }
  }

  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].state.position =
      dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]) / joint_gearing_[i];
    joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);

    const bool suspicious_position =
      std::abs(joints_[i].state.position) > kSuspiciousReportedPositionRad;
    const bool periodic_probe =
      !debug_joint_ids_.empty() && read_cycle_count_ % debug_every_n_reads_ == 0;

    if (is_debug_joint(ids[i]) && (suspicious_position || periodic_probe)) {
      int32_t direct_position = 0;
      int32_t direct_velocity = 0;
      int32_t direct_current = 0;

      const bool direct_position_ok =
        dynamixel_workbench_.itemRead(ids[i], kPresentPositionItem, &direct_position, &log);

      bool direct_velocity_ok =
        dynamixel_workbench_.itemRead(ids[i], kPresentVelocityItem, &direct_velocity, &log);
      if (!direct_velocity_ok) {
        direct_velocity_ok =
          dynamixel_workbench_.itemRead(ids[i], kPresentSpeedItem, &direct_velocity, &log);
      }

      bool direct_current_ok =
        dynamixel_workbench_.itemRead(ids[i], kPresentCurrentItem, &direct_current, &log);
      if (!direct_current_ok) {
        direct_current_ok =
          dynamixel_workbench_.itemRead(ids[i], kPresentLoadItem, &direct_current, &log);
      }

      RCLCPP_WARN(
        rclcpp::get_logger(kDynamixelHardware),
        "Probe joint '%s' ID=%d read#%llu: sync_raw(pos=%d vel=%d cur=%d) converted(pos=%f vel=%f effort=%f, gearing=%f) direct_raw(pos=%d vel=%d cur=%d) direct_ok(pos=%d vel=%d cur=%d)",
        info_.joints[i].name.c_str(), ids[i], static_cast<unsigned long long>(read_cycle_count_),
        positions[i], velocities[i], currents[i], joints_[i].state.position, joints_[i].state.velocity,
        joints_[i].state.effort, joint_gearing_[i], direct_position, direct_velocity, direct_current,
        direct_position_ok, direct_velocity_ok, direct_current_ok);

      if (!direct_position_ok || !direct_velocity_ok || !direct_current_ok) {
        RCLCPP_WARN(
          rclcpp::get_logger(kDynamixelHardware),
          "Direct read failed for joint '%s' (ID=%d). Last Dynamixel log: %s",
          info_.joints[i].name.c_str(), ids[i], log != nullptr ? log : "<none>");
      }
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
  std::vector<int32_t> commands;
  std::vector<uint8_t> ids;

  commands.reserve(info_.joints.size());
  ids.reserve(info_.joints.size());

  for (uint i = 0; i < joint_ids_.size(); i++) {
    const uint8_t id = joint_ids_[i];
    const bool command_changed =
      std::abs(joints_[i].command.position - joints_[i].prev_command.position) > 1e-9;
    const int32_t command = dynamixel_workbench_.convertRadian2Value(
      id, static_cast<float>(joints_[i].command.position * joint_gearing_[i]));

    if (is_direct_read_joint(id)) {
      if (command_changed) {
        if (!dynamixel_workbench_.itemWrite(id, kGoalPositionItem, command, &log)) {
          RCLCPP_ERROR(
            rclcpp::get_logger(kDynamixelHardware),
            "Direct goal-position write failed for ID=%d (%s)", id,
            log != nullptr ? log : "<none>");
          return CallbackReturn::ERROR;
        }
      }
    } else {
      ids.push_back(id);
      commands.push_back(command);
    }

    joints_[i].prev_command.position = joints_[i].command.position;

  }

  if (!ids.empty() && !dynamixel_workbench_.syncWrite(
                      kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::set_joint_velocities()
{
  const char * log = nullptr;
  std::vector<int32_t> commands(info_.joints.size(), 0);
  std::vector<uint8_t> ids(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
  for (uint i = 0; i < ids.size(); i++) {
    joints_[i].prev_command.velocity = joints_[i].command.velocity;
    commands[i] = dynamixel_workbench_.convertVelocity2Value(
      ids[i], static_cast<float>(joints_[i].command.velocity));
  }
  if (!dynamixel_workbench_.syncWrite(
        kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
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

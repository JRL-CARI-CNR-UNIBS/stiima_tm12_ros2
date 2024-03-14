// Copyright 2022
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  stefano.mutti@stiima.cnr.it
 * \date    2022-11-10
 *
 */
//----------------------------------------------------------------------
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "stiima_tm12_hw/hardware_interface.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace tm_robot_driver
{
hardware_interface::CallbackReturn
TMPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = system_info;

  // initialize
  // joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  // joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  // joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  // ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  // tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  // position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  // position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  // velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  stop_modes_ = { StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
                  StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE };
  start_modes_ = {};
  position_controller_running_ = false;
  velocity_controller_running_ = false;
  // runtime_state_ = static_cast<uint32_t>(rtde::RUNTIME_STATE::STOPPED);
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;
  controllers_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;
  // async_thread_shutdown_ = false;
  // system_interface_initialized_ = 0.0;

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("TMPositionHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("TMPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("TMPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("TMPositionHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("TMPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("TMPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("TMPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  #ifdef LOG_JOINTS
  package_share_directory_ = ament_index_cpp::get_package_share_directory("stiima_tm12_hw");
  RCLCPP_WARN(rclcpp::get_logger("TMPositionHardwareInterface"),"Creating log files in %s",package_share_directory_.c_str());
  jnt_target_.open (package_share_directory_ + "/jnt_target_.txt");
  jnt_actual_.open (package_share_directory_ + "/jnt_actual_.txt");
  #endif
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TMPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
  }

  // state_interfaces.emplace_back(
      // hardware_interface::StateInterface("speed_scaling", "speed_scaling_factor", &speed_scaling_combined_));

  // for (auto& sensor : info_.sensors) {
  //   for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
  //     state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
  //                                                                      &ft_sensor_measurements_[j]));
  //   }
  // }

  // for (size_t i = 0; i < 18; ++i) {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "digital_output_" + std::to_string(i),
  //                                                                    &actual_dig_out_bits_copy_[i]));
  //   state_interfaces.emplace_back(
  //       hardware_interface::StateInterface("gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_copy_[i]));
  // }

  // for (size_t i = 0; i < 11; ++i) {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "safety_status_bit_" + std::to_string(i),
  //                                                                    &safety_status_bits_copy_[i]));
  // }

  // for (size_t i = 0; i < 4; ++i) {
  //   state_interfaces.emplace_back(
  //       hardware_interface::StateInterface("gpio", "analog_io_type_" + std::to_string(i), &analog_io_types_copy_[i]));
  //   state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "robot_status_bit_" + std::to_string(i),
  //                                                                    &robot_status_bits_copy_[i]));
  // }

  // for (size_t i = 0; i < 2; ++i) {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //       "gpio", "tool_analog_input_type_" + std::to_string(i), &tool_analog_input_types_copy_[i]));

  //   state_interfaces.emplace_back(
  //       hardware_interface::StateInterface("gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input_[i]));

  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //       "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));

  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //       "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
  // }

  // state_interfaces.emplace_back(
  //     hardware_interface::StateInterface("gpio", "tool_output_voltage", &tool_output_voltage_copy_));

  // state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "robot_mode", &robot_mode_copy_));

  // state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "safety_mode", &safety_mode_copy_));

  // state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "tool_mode", &tool_mode_copy_));

  // state_interfaces.emplace_back(
  //     hardware_interface::StateInterface("gpio", "tool_output_current", &tool_output_current_));

  // state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "tool_temperature", &tool_temperature_));

  // state_interfaces.emplace_back(
      // hardware_interface::StateInterface("system_interface", "initialized", &system_interface_initialized_));

  // state_interfaces.emplace_back(
      // hardware_interface::StateInterface("gpio", "program_running", &robot_program_running_copy_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TMPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  // command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "io_async_success", &io_async_success_));

  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface("speed_scaling", "target_speed_fraction_cmd", &target_speed_fraction_cmd_));

  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     "speed_scaling", "target_speed_fraction_async_success", &scaling_async_success_));

  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     "resend_robot_program", "resend_robot_program_cmd", &resend_robot_program_cmd_));

  // command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //     "resend_robot_program", "resend_robot_program_async_success", &resend_robot_program_async_success_));

  // command_interfaces.emplace_back(hardware_interface::CommandInterface("payload", "mass", &payload_mass_));
  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface("payload", "cog.x", &payload_center_of_gravity_[0]));
  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface("payload", "cog.y", &payload_center_of_gravity_[1]));
  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface("payload", "cog.z", &payload_center_of_gravity_[2]));
  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface("payload", "payload_async_success", &payload_async_success_));

  // for (size_t i = 0; i < 18; ++i) {
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       "gpio", "standard_digital_output_cmd_" + std::to_string(i), &standard_dig_out_bits_cmd_[i]));
  // }

  // for (size_t i = 0; i < 2; ++i) {
  //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       "gpio", "standard_analog_output_cmd_" + std::to_string(i), &standard_analog_output_cmd_[i]));
  // }

  // command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "tool_voltage_cmd", &tool_voltage_cmd_));

  // command_interfaces.emplace_back(
  //     hardware_interface::CommandInterface("zero_ftsensor", "zero_ftsensor_cmd", &zero_ftsensor_cmd_));

  // command_interfaces.emplace_back(hardware_interface::CommandInterface("zero_ftsensor", "zero_ftsensor_async_success",
  //                                                                      &zero_ftsensor_async_success_));

  return command_interfaces;
}

hardware_interface::CallbackReturn
TMPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("TMPositionHardwareInterface"), "Starting ...please wait...");

  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  if(robot_ip.empty())
  {
    robot_ip = "169.254.49.101";
    RCLCPP_WARN(rclcpp::get_logger("TMPositionHardwareInterface"), "No IP found. Using default: %s", robot_ip.c_str());
  }
  // const std::string host = "169.254.49.101";

  RCLCPP_INFO(rclcpp::get_logger("TMPositionHardwareInterface"), "Initializing driver...");

  

  tm_driver_  = std::make_unique<TmDriver>(robot_ip, nullptr, nullptr);
  tm_svr_     = std::make_unique<TmSvrRos2>(*tm_driver_);
  tm_sct_     = std::make_unique<TmSctRos2>(*tm_driver_);
  

  // ur_driver_->startRTDECommunication();

  // async_thread_ = std::make_shared<std::thread>(&TMPositionHardwareInterface::asyncThread, this);

  RCLCPP_INFO(rclcpp::get_logger("TMPositionHardwareInterface"), "System successfully started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
TMPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("TMPositionHardwareInterface"), "Stopping ...please wait...");

  // async_thread_shutdown_ = true;
  // async_thread_->join();
  // async_thread_.reset();

  tm_sct_.reset();
  tm_svr_.reset();
  tm_driver_.reset();

  #ifdef LOG_JOINTS
  jnt_target_.close();
  jnt_actual_.close();
  #endif

  RCLCPP_INFO(rclcpp::get_logger("TMPositionHardwareInterface"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TMPositionHardwareInterface::read(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  tm_driver_->state.update_tm_robot_publish_state();
  if(tm_driver_->state.get_receive_state() != TmCommRC::TIMEOUT){

    //TODO check mutex
    //TODO send std vectors in , dont use returns
    std::vector<double> tmp_joint_positions_  = tm_driver_->state.joint_angle();
    std::vector<double> tmp_joint_velocities_ = tm_driver_->state.joint_speed();
    std::vector<double> tmp_joint_efforts_    = tm_driver_->state.joint_torque();

  for (size_t i=0; i<tmp_joint_positions_.size(); i++) 
  {
    joint_positions_[i]  = tmp_joint_positions_[i]; 
    joint_velocities_[i] = tmp_joint_velocities_[i]; 
    joint_efforts_[i]    = tmp_joint_efforts_[i]; 
  }

    // pm.fbs_msg.tool_pose = state.tool_pose();
    // pm.fbs_msg.tcp_speed = state.tcp_speed_vec();
    // pm.fbs_msg.tcp_force = state.tcp_force_vec();
    // pm.fbs_msg.robot_link = state.is_linked();
    // pm.fbs_msg.robot_error = state.has_error();
    // pm.fbs_msg.project_run = state.is_project_running();
    // pm.fbs_msg.project_pause = state.is_project_paused();
    // pm.fbs_msg.safetyguard_a = state.is_safeguard_A();
    // pm.fbs_msg.e_stop = state.is_EStop();
    // pm.fbs_msg.camera_light = state.camera_light();
    // pm.fbs_msg.error_code = state.error_code();
    // pm.fbs_msg.project_speed = state.project_speed();
    // pm.fbs_msg.ma_mode = state.ma_mode();
    // pm.fbs_msg.robot_light = state.robot_light();
    // pm.fbs_msg.cb_digital_output = state.ctrller_DO();
    // pm.fbs_msg.cb_digital_input = state.ctrller_DI();
    // pm.fbs_msg.cb_analog_output = state.ctrller_AO();
    // pm.fbs_msg.cb_analog_input = state.ctrller_AI();
    // pm.fbs_msg.ee_digital_output = state.ee_DO();
    // pm.fbs_msg.ee_digital_input = state.ee_DI();
    //pm.fbs_msg.ee_analog_output = state.ee_AO();
    // pm.fbs_msg.ee_analog_input = state.ee_AI();
    // pm.fbs_msg.error_content = state.error_content();

    // Publish torque state
    // pm.fbs_msg.joint_tor_average = state.joint_torque_average();
    // pm.fbs_msg.joint_tor_min = state.joint_torque_min();
    // pm.fbs_msg.joint_tor_max = state.joint_torque_max();

    // Publish joint state
    // pm.joint_msg.header.stamp = pm.fbs_msg.header.stamp;
    // pm.joint_msg.position = pm.fbs_msg.joint_pos;
    // pm.joint_msg.velocity = pm.fbs_msg.joint_vel;
    // pm.joint_msg.effort = pm.fbs_msg.joint_tor;

    // Publish tool pose
    // auto &pose = pm.fbs_msg.tool_pose;
    // tf2::Quaternion quat;
    // quat.setRPY(pose[3], pose[4], pose[5]);
    // pm.tool_pose_msg.header.stamp = pm.joint_msg.header.stamp;
    // pm.tool_pose_msg.pose.position.x = pose[0];
    // pm.tool_pose_msg.pose.position.y = pose[1];
    // pm.tool_pose_msg.pose.position.z = pose[2];
    // pm.tool_pose_msg.pose.orientation = tf2::toMsg(quat);


    if (first_pass_ && !initialized_) {
      position_commands_ = position_commands_old_ = joint_positions_;
      velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
      initialized_ = true;
    }

    // updateNonDoubleValues();

    #ifdef LOG_JOINTS
      jnt_actual_ << time.nanoseconds() << " ";
      for (size_t i=0; i<joint_positions_.size(); i++) 
      {
        jnt_actual_ << joint_positions_[i]  << " ";
        jnt_actual_ << joint_velocities_[i] << " ";
        jnt_actual_ << joint_efforts_[i]    << " ";
      }
      jnt_actual_ << "\n";
    #endif

    return hardware_interface::return_type::OK;
  }

  RCLCPP_ERROR(rclcpp::get_logger("TMPositionHardwareInterface"), "Unable to read from hardware...");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TMPositionHardwareInterface::write(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
{
  // if ((runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) ||
  //      runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSING)) &&
  //     robot_program_running_ && (!non_blocking_read_ || packet_read_)) {
    if (position_controller_running_) {
      // std::vector<double> position_commands_fake_ = { { 0.1, 0.2, 0.3, -0.1, -0.2, -0.3 } };
      // tm_driver_->set_joint_pos_PTP(position_commands_,2*M_PI,0.01,100,true);
      tm_driver_->set_joint_pos_PTP(position_commands_,0.3*M_PI,0.02,90,true);

    } else if (velocity_controller_running_) {
      RCLCPP_ERROR(rclcpp::get_logger("TMPositionHardwareInterface"), "No velocity control implemented yet");

    } else {
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("TMPositionHardwareInterface"), "No keepalive");
    }

    #ifdef LOG_JOINTS
      jnt_target_ << time.nanoseconds() << " ";
      for (size_t i=0; i<position_commands_.size(); i++) 
      {
        jnt_target_ << position_commands_[i]  << " ";
      }
      jnt_target_ << "\n";
    #endif

    packet_read_ = false;
  // }

  return hardware_interface::return_type::OK;
}

void TMPositionHardwareInterface::handleRobotProgramState(bool program_running)
{
  robot_program_running_ = program_running;
}


hardware_interface::return_type TMPositionHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  start_modes_.clear();
  stop_modes_.clear();

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        start_modes_.push_back(hardware_interface::HW_IF_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
      }
    }
  }
  // set new mode to all interfaces at the same time
  if (start_modes_.size() != 0 && start_modes_.size() != 6) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // all start interfaces must be the same - can't mix position and velocity control
  if (start_modes_.size() != 0 && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        stop_modes_.push_back(StoppingInterface::STOP_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
      }
    }
  }
  // stop all interfaces at the same time
  if (stop_modes_.size() != 0 &&
      (stop_modes_.size() != 6 || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  controllers_initialized_ = true;
  return ret_val;
}

hardware_interface::return_type TMPositionHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_modes_.size() != 0 &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION) != stop_modes_.end()) {
    position_controller_running_ = false;
    position_commands_ = position_commands_old_ = joint_positions_;
  } else if (stop_modes_.size() != 0 &&
             std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY) != stop_modes_.end()) {
    velocity_controller_running_ = false;
    velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  }

  if (start_modes_.size() != 0 &&
      std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) {
    velocity_controller_running_ = false;
    position_commands_ = position_commands_old_ = joint_positions_;
    position_controller_running_ = true;

  } else if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(),
                                                   hardware_interface::HW_IF_VELOCITY) != start_modes_.end()) {
    position_controller_running_ = false;
    velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    velocity_controller_running_ = true;
  }

  start_modes_.clear();
  stop_modes_.clear();

  return ret_val;
}
}  // namespace tm_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tm_robot_driver::TMPositionHardwareInterface, hardware_interface::SystemInterface)

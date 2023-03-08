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
#ifndef TM_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define TM_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <iostream>
#include <fstream>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

// TM stuff
#include "stiima_tm12_hw/tm_driver.h"
#include "stiima_tm12_hw/tm_ros2_sct.h"
#include "stiima_tm12_hw/tm_ros2_svr.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tm_robot_driver
{
enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
};

enum StoppingInterface
{
  NONE,
  STOP_POSITION,
  STOP_VELOCITY
};

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class TMPositionHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TMPositionHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;

  /*!
   * \brief Callback to handle a change in the current state of the URCaps program running on the
   * robot. Executed only on the state change.
   *
   * \param program_running True when the URCap program is running on the robot.
   */
  void handleRobotProgramState(bool program_running);

  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

  void asyncThread();

protected:

  void initAsyncIO();
  void checkAsyncIO();
  void updateNonDoubleValues();
  void extractToolPose();
  void transformForceTorque();

  std::vector<double>    joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  std::vector<double>    joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  std::vector<double>    joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  std::vector<double>    ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  std::vector<double>    tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  std::vector<double>    position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  std::vector<double>    position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  std::vector<double>    velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  #ifdef LOG_JOINTS
  std::string package_share_directory_;
  std::ofstream jnt_target_;
  std::ofstream jnt_actual_;
  #endif

  bool packet_read_;

  uint32_t runtime_state_;
  bool controllers_initialized_;

  // std::bitset<18> actual_dig_out_bits_;
  // std::bitset<18> actual_dig_in_bits_;
  // std::array<double, 2> standard_analog_input_;
  // std::array<double, 2> standard_analog_output_;
  // std::bitset<4> analog_io_types_;
  // uint32_t tool_mode_;
  // std::bitset<2> tool_analog_input_types_;
  // std::array<double, 2> tool_analog_input_;
  // int32_t tool_output_voltage_;
  // double tool_output_current_;
  // double tool_temperature_;
  // double speed_scaling_;
  // double target_speed_fraction_;
  // double speed_scaling_combined_;
  // int32_t robot_mode_;
  // int32_t safety_mode_;
  // std::bitset<4> robot_status_bits_;
  // std::bitset<11> safety_status_bits_;

  // transform stuff
  // tf2::Vector3 tcp_force_;
  // tf2::Vector3 tcp_torque_;
  // geometry_msgs::msg::TransformStamped tcp_transform_;

  // asynchronous commands
  // std::array<double, 18> standard_dig_out_bits_cmd_;
  // std::array<double, 2> standard_analog_output_cmd_;
  // double tool_voltage_cmd_;
  // double io_async_success_;
  // double target_speed_fraction_cmd_;
  // double scaling_async_success_;
  // double resend_robot_program_cmd_;
  // double resend_robot_program_async_success_;
  // double zero_ftsensor_cmd_;
  // double zero_ftsensor_async_success_;
  bool first_pass_;
  bool initialized_;
  // double system_interface_initialized_;
  // bool async_thread_shutdown_;

  // payload stuff
  // urcl::vector3d_t payload_center_of_gravity_;
  // double payload_mass_;
  // double payload_async_success_;

  // copy of non double values
  // std::array<double, 18> actual_dig_out_bits_copy_;
  // std::array<double, 18> actual_dig_in_bits_copy_;
  // std::array<double, 4> analog_io_types_copy_;
  // double tool_mode_copy_;
  // std::array<double, 2> tool_analog_input_types_copy_;
  // double tool_output_voltage_copy_;
  // double robot_mode_copy_;
  // double safety_mode_copy_;
  // std::array<double, 4> robot_status_bits_copy_;
  // std::array<double, 11> safety_status_bits_copy_;

  bool robot_program_running_;
  bool non_blocking_read_;
  double robot_program_running_copy_;

  PausingState pausing_state_;
  double pausing_ramp_up_increment_;

  // resources switching aux vars
  std::vector<uint> stop_modes_;
  std::vector<std::string> start_modes_;
  bool position_controller_running_;
  bool velocity_controller_running_;

  std::unique_ptr<TmDriver>           tm_driver_;
  std::unique_ptr<TmSvrRos2>          tm_svr_;
  std::unique_ptr<TmSctRos2>          tm_sct_;
  // std::shared_ptr<std::thread> async_thread_;
};
}  // namespace tm_robot_driver

#endif  // TM_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

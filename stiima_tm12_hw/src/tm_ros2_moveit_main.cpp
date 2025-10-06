#include "stiima_tm12_hw/tm_ros2_moveit.hpp"
#include "stiima_tm12_hw/tm_ros2_svr.h"
#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tm_moveit_standalone");
  node->declare_parameter("joints", 
                                std::vector<std::string>({"joint_1", 
                                                          "joint_2", 
                                                          "joint_3", 
                                                          "joint_4", 
                                                          "joint_5", 
                                                          "joint_6"}));
  node->declare_parameter("robot_ip", "169.254.49.101");
  std::string robot_ip = "";
  if(!node->get_parameter("robot_ip", robot_ip)) {
    RCLCPP_WARN(node->get_logger(), "Failed to get robot_ip parameter, using default.");
  }
  std::vector<std::string> joints;
  if(!node->get_parameter("joints", joints)) {
    RCLCPP_WARN(node->get_logger(), "Failed to get joints parameter, using default.");
  }
  std::string action_name;
  node->declare_parameter("action_name", 
                          std::string("tmr_arm_controller/follow_joint_trajectory"));
  if(!node->get_parameter("action_name", action_name)) {
    RCLCPP_WARN(node->get_logger(), "Failed to get action_name parameter, using default.");
  }

  auto tm_driver  = std::make_unique<TmDriver>(robot_ip, nullptr, nullptr);
   
  auto tm_svr = std::make_shared<TmSvrRos2>(*tm_driver, true);

  auto node_moveit_driver_interface = std::make_shared<TmRos2SctMoveit>(*tm_driver, action_name, joints);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(tm_svr);
  executor.add_node(node_moveit_driver_interface);
  tm_driver->set_tag(1, 0);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}

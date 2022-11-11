/*******************************************************************************
 * Copyright (c) 2022/11/10, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "left_front_wheel_controller.h"

#include <std_msgs/Float64.h>

#include <pluginlib/class_list_macros.hpp>
namespace rm_control_study {
bool LeftFrontWheelController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                                    ros::NodeHandle &controller_nh) {
  cmd_subscriber_ = root_nh.subscribe<std_msgs::Float64>("lf_command", 1, &LeftFrontWheelController::commandCB, this);

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (not ctrl_lf_.init(effort_joint_interface_, controller_nh)) {
    return false;
  }
  return true;
}

void LeftFrontWheelController::update(const ros::Time &time, const ros::Duration &period) {
  double vel = ctrl_lf_.joint_.getVelocity();
  ROS_INFO("vel is : %f", vel);
  ctrl_lf_.setCommand(lf_joint_cmd_);
  ctrl_lf_.update(time, period);
}
void LeftFrontWheelController::commandCB(const std_msgs::Float64::ConstPtr &msg) { lf_joint_cmd_ = msg->data; }
}  // namespace rm_control_study

PLUGINLIB_EXPORT_CLASS(rm_control_study::LeftFrontWheelController, controller_interface::ControllerBase)

/*******************************************************************************
 * Copyright (c) 2023/04/23, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "joint1_vel_controller.h"

#include <std_msgs/Float64.h>

#include <pluginlib/class_list_macros.hpp>
namespace rm_control_study {
bool Joint1VelController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
                               ros::NodeHandle &controller_nh) {
  cmd_subscriber_ =
      root_nh.subscribe<std_msgs::Float64>("joint1_vel_command", 1, &Joint1VelController::commandCB, this);

  effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (not ctrl_joint1_.init(effort_joint_interface_, controller_nh)) {
    return false;
  }

  joint1_state_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(root_nh, "joint_state", 100));

  return true;
}
void Joint1VelController::update(const ros::Time &time, const ros::Duration &period) {
  double vel = ctrl_joint1_.joint_.getVelocity();
  if (joint1_state_pub_->trylock()) {
    joint1_state_pub_->msg_.data = vel;
    joint1_state_pub_->unlockAndPublish();
  }
  ctrl_joint1_.setCommand(joint1_cmd_);
  ctrl_joint1_.update(time, period);
}
void Joint1VelController::commandCB(const std_msgs::Float64::ConstPtr &msg) { joint1_cmd_ = msg->data; }
}  // namespace rm_control_study

PLUGINLIB_EXPORT_CLASS(rm_control_study::Joint1VelController, controller_interface::ControllerBase)

/*******************************************************************************
 * Copyright (c) 2022/11/10, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/ros_utilities.h>

#include <utility>

namespace rm_control_study {

class LeftFrontWheelController
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                            rm_control::RobotStateInterface> {
 public:
  LeftFrontWheelController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  void commandCB(const std_msgs::Float64::ConstPtr& msg);

  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  effort_controllers::JointVelocityController ctrl_lf_;
  ros::Subscriber cmd_subscriber_;
  double lf_joint_cmd_{};
};

}  // namespace rm_control_study

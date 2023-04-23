/*******************************************************************************
 * Copyright (c) 2023/04/23, Liao LunJia.
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

class Joint1VelController
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                            rm_control::RobotStateInterface> {
 public:
  Joint1VelController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  void commandCB(const std_msgs::Float64::ConstPtr& msg);
  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  effort_controllers::JointVelocityController ctrl_joint1_;
  ros::Subscriber cmd_subscriber_;
  double joint1_cmd_{};
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> > joint1_state_pub_;
};

}  // namespace rm_control_study

#ifndef FORWARD_COMMAND_CONTROLLER_FORWARD_COMMAND_CONTROLLER_H
#define FORWARD_COMMAND_CONTROLLER_FORWARD_COMMAND_CONTROLLER_H

#include <ros/ros.h>
#include <robot_drive_controllers/robot_drive_controller.h>
#include <river_ros_util/ros_util.h>

namespace robot_drive_controllers
{

class Skid4WheelDriveController: public RobotDriveController<hardware_interface::EffortJointInterface>
{
private:
  double rotations_per_meter;
  double base_width;
  double base_length;
public:

  Skid4WheelDriveController() : rotations_per_meter(1.0), base_width(0.554), base_length(0.52) {}


  bool initJoints(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
	river_ros_util::get_param(n, rotations_per_meter, "rotations_per_meter");
	river_ros_util::get_param(n, base_width, "base_width");
	river_ros_util::get_param(n, base_length, "base_length");

	define_and_get_param(n, std::string, front_left_joint_name, "front_left_joint_name", "");
	define_and_get_param(n, std::string, front_right_joint_name, "front_right_joint_name", "");
	define_and_get_param(n, std::string, back_left_joint_name, "back_left_joint_name", "");
	define_and_get_param(n, std::string, back_right_joint_name, "back_right_joint_name", "");
    front_left_joint_ = hw->getHandle(front_left_joint_name);
    front_right_joint_ = hw->getHandle(front_right_joint_name);
    back_left_joint_ = hw->getHandle(back_left_joint_name);
    back_right_joint_ = hw->getHandle(back_right_joint_name);
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period) {
	  double u = command_->linear.x;
	  double w = command_->angular.z;
	  double r = sqrt(base_length*base_length/4 + base_width*base_width/4);//distance from turning center to wheel
	  double u1 = u - 2 * r*r / base_width * w;
	  double u2 = u + 2 * r*r / base_width * w;

	  double leftSpeed = u1*rotations_per_meter;//rps
	  double rightSpeed = u2*rotations_per_meter;//rps

	  front_left_joint_.setCommand(leftSpeed);
	  back_left_joint_.setCommand(leftSpeed);
	  front_right_joint_.setCommand(rightSpeed);
	  back_right_joint_.setCommand(rightSpeed);
  }

  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;
};

}

#endif

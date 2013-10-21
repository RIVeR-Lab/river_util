#ifndef ROBOT_DRIVE_CONTROLLERS_SKID_4_WHEEL_DRIVE_CONTROLLER_H
#define ROBOT_DRIVE_CONTROLLERS_SKID_4_WHEEL_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <robot_drive_controllers/robot_drive_controller.h>
#include <river_ros_util/ros_util.h>

namespace robot_drive_controllers
{

class Skid4WheelDriveController: public RobotDriveController<hardware_interface::VelocityJointInterface>
{
private:
  double rotations_per_meter;
  double base_width;
  double base_length;
public:

  Skid4WheelDriveController() : rotations_per_meter(1.0), base_width(0.554), base_length(0.52) {}


  bool initJoints(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
  {
    river_ros_util::get_param(n, rotations_per_meter, "rotations_per_meter");
    river_ros_util::get_param(n, base_width, "base_width");
    river_ros_util::get_param(n, base_length, "base_length");

    define_and_get_param(n, std::string, left_joint_name, "left_joint_name", "");
    define_and_get_param(n, std::string, right_joint_name, "right_joint_name", "");

    left_joint_ = hw->getHandle(left_joint_name);
    right_joint_ = hw->getHandle(right_joint_name);

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

	  left_joint_.setCommand(leftSpeed);
	  right_joint_.setCommand(rightSpeed);
  }

  hardware_interface::JointHandle left_joint_;
  hardware_interface::JointHandle right_joint_;
};

}

#endif

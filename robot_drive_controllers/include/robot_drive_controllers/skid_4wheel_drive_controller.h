#ifndef ROBOT_DRIVE_CONTROLLERS_SKID_4_WHEEL_DRIVE_CONTROLLER_H
#define ROBOT_DRIVE_CONTROLLERS_SKID_4_WHEEL_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <robot_drive_controllers/robot_drive_controller.h>
#include <river_ros_util/ros_util.h>
#include <realtime_tools/realtime_publisher.h>
#include <math.h>

namespace robot_drive_controllers
{

class Skid4WheelDriveController: public RobotDriveController<hardware_interface::VelocityJointInterface>
{
private:
  double rotations_per_meter;
  double base_width;
  double base_length;
public:

  Skid4WheelDriveController();

  bool initJoints(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n);

  void update(const ros::Time& time, const ros::Duration& period);

  hardware_interface::JointHandle left_joint_;
  hardware_interface::JointHandle right_joint_;
};

}

#endif

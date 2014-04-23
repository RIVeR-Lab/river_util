#ifndef ROBOT_DRIVE_CONTROLLERS_SKID_4_WHEEL_DRIVE_CONTROLLER_H
#define ROBOT_DRIVE_CONTROLLERS_SKID_4_WHEEL_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <robot_drive_controllers/robot_drive_controller.h>
#include <river_ros_util/ros_util.h>
#include <std_msgs/Float64.h>
#include <math.h>

namespace robot_drive_controllers
{

class Skid4WheelDriveController: public RobotDriveController
{
private:
  double rotations_per_meter;
  double base_width;
  double base_length;
  ros::Publisher left_vel_pub;
  ros::Publisher right_vel_pub;

public:

  Skid4WheelDriveController();

  bool initJoints(ros::NodeHandle &n, ros::NodeHandle &pn, std::vector<std::string>& joints);

  bool update_odom(const ros::Duration& period, const std::vector<double>& pos, const std::vector<double>& vel, const std::vector<double>& eff);
  void update_cmd(const geometry_msgs::TwistConstPtr& msg);
};

}

#endif

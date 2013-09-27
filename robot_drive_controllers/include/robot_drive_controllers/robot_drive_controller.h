#ifndef ROBOT_DRIVE_CONTROLLER_ROBOT_DRIVE_CONTROLLER_H
#define ROBOT_DRIVE_CONTROLLER_ROBOT_DRIVE_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <river_ros_util/ros_util.h>


namespace robot_drive_controllers
{

template <class T>
class RobotDriveController: public controller_interface::Controller<T>
{
public:
  RobotDriveController() : command_(geometry_msgs::TwistConstPtr(new geometry_msgs::Twist())) {}
  virtual ~RobotDriveController() {sub_command_.shutdown();}

  virtual bool initJoints(T* hw, ros::NodeHandle &n) = 0;

  bool init(T* hw, ros::NodeHandle &n)
  {
    bool success = initJoints(hw, n);
    if(!success)
      return false;
    sub_command_ = n.subscribe<geometry_msgs::Twist>("command", 1, &RobotDriveController::commandCB, this);
    return true;
  }

  void starting(const ros::Time& time) {command_ = geometry_msgs::TwistConstPtr(new geometry_msgs::Twist());}

  geometry_msgs::TwistConstPtr command_;

private:
  ros::Subscriber sub_command_;
  void commandCB(const geometry_msgs::TwistConstPtr& msg) {command_ = msg;}
};

}

#endif

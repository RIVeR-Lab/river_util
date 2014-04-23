#include <robot_drive_controllers/skid_4wheel_drive_controller.h>
#include <ros/ros.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "skid_4wheel_drive_controller");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  robot_drive_controllers::Skid4WheelDriveController controller;
  controller.init(n, pn);
  ros::spin();
}

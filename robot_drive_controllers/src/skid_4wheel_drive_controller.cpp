#include <robot_drive_controllers/skid_4wheel_drive_controller.h>
#include <pluginlib/class_list_macros.h>


namespace robot_drive_controllers
{

  Skid4WheelDriveController::Skid4WheelDriveController() : rotations_per_meter(1.0), base_width(0.554), base_length(0.52) {}


  bool Skid4WheelDriveController::initJoints(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
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

  void Skid4WheelDriveController::update(const ros::Time& time, const ros::Duration& period) {
	  double r = sqrt(base_length*base_length/4 + base_width*base_width/4);//distance from turning center to wheel


	  double odom_new_u = (left_joint_.getVelocity() + right_joint_.getVelocity())/2;
	  double odom_new_w = (right_joint_.getVelocity() - left_joint_.getVelocity())*base_width/(4*r*r);
	  double dt = period.toSec();
	  double dx, dy;
	  if(odom_new_w!=0){
		  double a = odom_new_w*dt;//angle rotated
		  double r = odom_new_u/odom_new_w;//radius of arc driven

		  dx = r*sin(odom_theta+a) - r*sin(odom_theta);
		  dy = -r*cos(odom_theta+a) + r*cos(odom_theta);
	  }
	  else{//w==0
		  double d = odom_new_u*dt;
		  dx = d*cos(odom_theta);
		  dy = d*sin(odom_theta);
	  }

	  odom_x += dx;
	  odom_y += dy;

	  odom_theta += odom_w*dt;
	  odom_u = odom_new_u;
	  odom_w = odom_new_w;
	  odom_time = time;
	  odom_updated();


	  double u = command_->linear.x;
	  double w = command_->angular.z;
	  double u1 = u - 2 * r*r / base_width * w;
	  double u2 = u + 2 * r*r / base_width * w;

	  double leftSpeed = u1*rotations_per_meter;//rps
	  double rightSpeed = u2*rotations_per_meter;//rps

	  left_joint_.setCommand(leftSpeed*2*M_PI);
	  right_joint_.setCommand(rightSpeed*2*M_PI);
  }

}

PLUGINLIB_EXPORT_CLASS(robot_drive_controllers::Skid4WheelDriveController, controller_interface::ControllerBase)

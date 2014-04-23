#include <robot_drive_controllers/skid_4wheel_drive_controller.h>


namespace robot_drive_controllers
{

  Skid4WheelDriveController::Skid4WheelDriveController() : rotations_per_meter(1.0), base_width(0.554), base_length(0.52) {}


  bool Skid4WheelDriveController::initJoints(ros::NodeHandle &n, ros::NodeHandle &pn, std::vector<std::string>& joints)
  {
    river_ros_util::get_param(pn, rotations_per_meter, "rotations_per_meter");
    river_ros_util::get_param(pn, base_width, "base_width");
    river_ros_util::get_param(pn, base_length, "base_length");

    define_and_get_param(pn, std::string, left_joint_name, "left_joint_name", "left_wheel_joint");
    define_and_get_param(pn, std::string, right_joint_name, "right_joint_name", "right_wheel_joint");
    joints.push_back(left_joint_name);
    joints.push_back(right_joint_name);

    left_vel_pub = n.advertise<std_msgs::Float64>("left_drive", 1);
    right_vel_pub = n.advertise<std_msgs::Float64>("right_drive", 1);

    return true;
  }


  void Skid4WheelDriveController::update_cmd(const geometry_msgs::TwistConstPtr& command){
	  double r = sqrt(base_length*base_length/4 + base_width*base_width/4);//distance from turning center to wheel
	  double u = command->linear.x;
	  double w = command->angular.z;
	  double u1 = u - 2 * r*r / base_width * w;
	  double u2 = u + 2 * r*r / base_width * w;

	  double leftSpeed = u1*rotations_per_meter;//rps
	  double rightSpeed = u2*rotations_per_meter;//rps

	  std_msgs::Float64 left_msg;
          left_msg.data = leftSpeed*2*M_PI;
	  std_msgs::Float64 right_msg;
          right_msg.data = rightSpeed*2*M_PI;
	  left_vel_pub.publish(left_msg);
	  right_vel_pub.publish(right_msg);
  }

  bool Skid4WheelDriveController::update_odom(const ros::Duration& period, const std::vector<double>& pos, const std::vector<double>& vel, const std::vector<double>& eff) {
	  double r = sqrt(base_length*base_length/4 + base_width*base_width/4);//distance from turning center to wheel
          double left_vel = vel[0];
          double right_vel = vel[1];
	  
	  double odom_u1 = left_vel/(2*M_PI*rotations_per_meter);
	  double odom_u2 = right_vel/(2*M_PI*rotations_per_meter);

	  double odom_new_u = (odom_u1+odom_u2)/2;
	  double odom_new_w = (odom_u2-odom_u1)*base_width/(4*r*r);
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

          return true;
  }

}

#ifndef ROBOT_DRIVE_CONTROLLER_ROBOT_DRIVE_CONTROLLER_H
#define ROBOT_DRIVE_CONTROLLER_ROBOT_DRIVE_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <river_ros_util/ros_util.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <vector>
#include <boost/foreach.hpp>

namespace robot_drive_controllers
{

class RobotDriveController
{
private:
	std::string robot_frame_;
	std::string odom_frame_;
	double odom_publish_rate_;
public:
 RobotDriveController():
  robot_frame_("base_footprint"), odom_frame_("odom"),
  odom_publish_rate_(50), last_publish_time(0),
  odom_time(0),
  odom_x(0), odom_y(0), odom_theta(0), odom_u(0), odom_w(0){}

  virtual bool initJoints(ros::NodeHandle &n, ros::NodeHandle &pn, std::vector<std::string>& joints) = 0;

  bool init(ros::NodeHandle &n, ros::NodeHandle &pn)
  {
	  river_ros_util::get_param(pn, robot_frame_, "robot_frame");
	  river_ros_util::get_param(pn, odom_frame_, "odom_frame");
	  river_ros_util::get_param(pn, odom_publish_rate_, "odom_publish_rate");

	  bool success = initJoints(n, pn, joints);
	  if(!success)
		  return false;

	  command_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &RobotDriveController::update_cmd, this);
	  joint_sub = n.subscribe<sensor_msgs::JointState>("joint_states", 1, &RobotDriveController::jointCB, this);
	  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);

	  return true;
  }

protected:
  double odom_x;
  double odom_y;
  double odom_theta;
  double odom_u;
  double odom_w;
  ros::Time odom_time;
  ros::Time last_publish_time;

  virtual bool update_odom(const ros::Duration& period, const std::vector<double>& pos, const std::vector<double>& vel, const std::vector<double>& eff) = 0;
  virtual void update_cmd(const geometry_msgs::TwistConstPtr& msg) = 0;



private:
  ros::Subscriber command_sub;
  ros::Subscriber joint_sub;
  ros::Publisher odom_pub;
  std::vector<std::string> joints;
  void jointCB(const sensor_msgs::JointStateConstPtr& msg) {
    if(odom_time.is_zero()){
      odom_time = msg->header.stamp;
    }
    else{
      std::vector<double> joint_pos;
      std::vector<double> joint_vel;
      std::vector<double> joint_eff;
      BOOST_FOREACH(std::string joint_name, joints){
	std::vector<std::string>::const_iterator joint_itr = std::find(msg->name.begin(), msg->name.end(), joint_name);
	if(joint_itr == msg->name.end()){
	  ROS_WARN_STREAM("Got joint message that is missing requested joint: "<<joint_name);
	  return;
	}
	int index = joint_itr - msg->name.begin();
	joint_pos.push_back(msg->position[index]);
	joint_vel.push_back(msg->velocity[index]);
	joint_eff.push_back(msg->effort[index]);
      }
      ros::Time current_time = msg->header.stamp;
      ros::Duration period = current_time - odom_time;
      if(update_odom(period, joint_pos, joint_vel, joint_eff))
        odom_updated();
      odom_time = current_time;
    }
  }
  void odom_updated(){
	if(last_publish_time + ros::Duration(1/odom_publish_rate_) < odom_time){
          nav_msgs::Odometry msg;
	  msg.pose.covariance.assign(0);
	  msg.pose.covariance[0] = 0.1;
	  msg.pose.covariance[7] = 0.1;
	  msg.pose.covariance[14] = 99999;
	  msg.pose.covariance[21] = 99999;
	  msg.pose.covariance[28] = 99999;
	  msg.pose.covariance[35] = 10;

	  msg.twist.covariance.assign(0);
	  msg.twist.covariance[0] = 0.1;
	  msg.twist.covariance[7] = 99999;
	  msg.twist.covariance[14] = 99999;
	  msg.twist.covariance[21] = 99999;
	  msg.twist.covariance[28] = 99999;
	  msg.twist.covariance[35] = 10;

	  msg.header.frame_id = odom_frame_;
	  msg.child_frame_id = robot_frame_;

	  msg.header.stamp = odom_time;
	  msg.pose.pose.position.x = odom_x;
	  msg.pose.pose.position.y = odom_y;
	  msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_theta);

	  msg.twist.twist.linear.x = odom_u;
	  msg.twist.twist.angular.z = odom_w;
	  odom_pub.publish(msg);
	  last_publish_time += ros::Duration(1/odom_publish_rate_);
	}
  }
};

}

#endif

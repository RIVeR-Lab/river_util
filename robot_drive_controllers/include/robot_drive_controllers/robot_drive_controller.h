#ifndef ROBOT_DRIVE_CONTROLLER_ROBOT_DRIVE_CONTROLLER_H
#define ROBOT_DRIVE_CONTROLLER_ROBOT_DRIVE_CONTROLLER_H

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <river_ros_util/ros_util.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tf.h>

namespace robot_drive_controllers
{

template <class T>
class RobotDriveController: public controller_interface::Controller<T>
{
private:
	std::string robot_frame_;
	std::string odom_frame_;
	std::string odom_topic_;
	std::string command_topic_;
	double odom_publish_rate_;
public:
  RobotDriveController() : command_(geometry_msgs::TwistConstPtr(new geometry_msgs::Twist())),
  robot_frame_("base_footprint"), odom_frame_("odom"),
  odom_topic_("odom"), command_topic_("command"), odom_publish_rate_(50), last_publish_time(0),
  odom_x(0), odom_y(0), odom_theta(0), odom_u(0), odom_w(0){}
  virtual ~RobotDriveController() {sub_command_.shutdown();/*pub_odom_->shutdown();*/}

  virtual bool initJoints(T* hw, ros::NodeHandle &n) = 0;

  bool init(T* hw, ros::NodeHandle &n)
  {
	  river_ros_util::get_param(n, robot_frame_, "robot_frame");
	  river_ros_util::get_param(n, odom_frame_, "odom_frame");
	  river_ros_util::get_param(n, odom_publish_rate_, "odom_publish_rate");
	  river_ros_util::get_param(n, odom_topic_, "odom_topic");
	  river_ros_util::get_param(n, command_topic_, "command_topic");

	  bool success = initJoints(hw, n);
	  if(!success)
		  return false;

	  sub_command_ = n.subscribe<geometry_msgs::Twist>(command_topic_, 1, &RobotDriveController::commandCB, this);
	  pub_odom_ = boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> >(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(n, odom_topic_, 100));

	  pub_odom_->msg_.pose.covariance.assign(0);
	  pub_odom_->msg_.pose.covariance[0] = 0.1;
	  pub_odom_->msg_.pose.covariance[7] = 0.1;
	  pub_odom_->msg_.pose.covariance[14] = 99999;
	  pub_odom_->msg_.pose.covariance[21] = 99999;
	  pub_odom_->msg_.pose.covariance[28] = 99999;
	  pub_odom_->msg_.pose.covariance[35] = 10;

	  pub_odom_->msg_.twist.covariance.assign(0);
	  pub_odom_->msg_.twist.covariance[0] = 0.1;
	  pub_odom_->msg_.twist.covariance[7] = 99999;
	  pub_odom_->msg_.twist.covariance[14] = 99999;
	  pub_odom_->msg_.twist.covariance[21] = 99999;
	  pub_odom_->msg_.twist.covariance[28] = 99999;
	  pub_odom_->msg_.twist.covariance[35] = 10;

	  pub_odom_->msg_.header.frame_id = odom_frame_;
	  pub_odom_->msg_.child_frame_id = robot_frame_;

	  return true;
  }

  void starting(const ros::Time& time) {command_ = geometry_msgs::TwistConstPtr(new geometry_msgs::Twist());}


protected:
  double odom_x;
  double odom_y;
  double odom_theta;
  double odom_u;
  double odom_w;
  ros::Time odom_time;
  ros::Time last_publish_time;

  void odom_updated(){
	  if(last_publish_time + ros::Duration(1/odom_publish_rate_) < odom_time){
		  if(pub_odom_->trylock())
		  {
			  pub_odom_->msg_.header.stamp = odom_time;
			  pub_odom_->msg_.pose.pose.position.x = odom_x;
			  pub_odom_->msg_.pose.pose.position.y = odom_y;
			  pub_odom_->msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_theta);

			  pub_odom_->msg_.twist.twist.linear.x = odom_u;
			  pub_odom_->msg_.twist.twist.angular.z = odom_w;
			  pub_odom_->unlockAndPublish();
			  last_publish_time += ros::Duration(1/odom_publish_rate_);
		  }
	  }
  }

protected:
  geometry_msgs::TwistConstPtr command_;

private:
  ros::Subscriber sub_command_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > pub_odom_;
  void commandCB(const geometry_msgs::TwistConstPtr& msg) {command_ = msg;}
};

}

#endif

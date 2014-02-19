#include "stereo_image_sync/stereo_image_sync.h"

namespace stereo_image_sync{

  StereoImageSync::StereoImageSync(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh),
									     updater(nh, pnh){
    ROS_INFO("Initializing Stereo Image Sync");

    define_and_get_param(pnh_, std::string, left_in_topic, "left_in", "left_in");
    define_and_get_param(pnh_, std::string, right_in_topic, "right_in", "right_in");

    define_and_get_param(pnh_, std::string, left_out_topic, "left_out", "left_out");
    define_and_get_param(pnh_, std::string, right_out_topic, "right_out", "right_out");

    define_and_get_param(pnh_, double, camera_rate, "camera_rate", 15);
    define_and_get_param(pnh_, double, camera_rate_threshold, "camera_rate_threshold", 0.75);
    max_time_diff_ = 1/camera_rate * camera_rate_threshold;

    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));

    left_pub_ = it_->advertiseCamera(left_out_topic, 1);
    right_pub_ = it_->advertiseCamera(right_out_topic, 1);

    left_sub_ = it_->subscribeCamera(left_in_topic, 1, &StereoImageSync::left_callback, this);
    right_sub_ = it_->subscribeCamera(right_in_topic, 1, &StereoImageSync::right_callback, this);

    updater.setHardwareID("none");
    updater.add("Function updater", this, &StereoImageSync::update_diagnostic_status);
    updater.force_update();
  }
  void StereoImageSync::update_diagnostic_status(diagnostic_updater::DiagnosticStatusWrapper &stat){
    stat.add("Mean Frame Time Diff", mean(stats_acc));
    stat.add("Min Frame Time Diff", min(stats_acc));
    stat.add("Max Frame Time Diff", max(stats_acc));
  }


  void StereoImageSync::process_input(){
    if(last_left_ && last_right_){
      double timediff = (last_left_->header.stamp - last_right_->header.stamp).toSec();
      if(abs(timediff)<max_time_diff_){
	sensor_msgs::Image* mod_right = new sensor_msgs::Image(*last_right_);
	mod_right->header.stamp = last_left_->header.stamp;
	mod_right->header.seq = last_left_->header.seq;
	sensor_msgs::ImageConstPtr mod_right_ptr(mod_right);

	sensor_msgs::CameraInfo* mod_right_info = new sensor_msgs::CameraInfo(*last_right_info_);
	mod_right_info->header.stamp = mod_right_ptr->header.stamp;
	mod_right_info->header.seq = mod_right_ptr->header.seq;
	sensor_msgs::CameraInfoConstPtr mod_right_info_ptr(mod_right_info);

	left_pub_.publish(last_left_, last_left_info_);
	right_pub_.publish(mod_right_ptr, mod_right_info_ptr);

	last_left_info_ = nullCameraInfoPtr;
	last_left_ = nullImagePtr;
	last_right_info_ = nullCameraInfoPtr;
	last_right_ = nullImagePtr;

	stats_acc(timediff);
      }
      else if(timediff<0){//right image much older than left
	ROS_WARN_THROTTLE(0.5, "Dropping right image (too old), Missed by: %fs", abs(timediff)-max_time_diff_);
	last_right_info_ = nullCameraInfoPtr;
	last_right_ = nullImagePtr;
      }
      else if(timediff>0){//left image much older than right
	ROS_WARN_THROTTLE(0.5, "Dropping left image (too old), Missed by: %fs", abs(timediff)-max_time_diff_);
	last_left_info_ = nullCameraInfoPtr;
	last_left_ = nullImagePtr;
      }
    }
    updater.update();
  }

  void StereoImageSync::left_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info){
    if(last_left_)
      ROS_WARN_THROTTLE(0.5, "Dropping left image (did not get right before next left)");
    last_left_ = msg;
    last_left_info_ = info;
    process_input();
  }

  void StereoImageSync::right_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info){
    if(last_right_)
      ROS_WARN_THROTTLE(0.5, "Dropping right image (did not get left before next right)");
    last_right_ = msg;
    last_right_info_ = info;
    process_input();
  }

}

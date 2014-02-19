#ifndef STEREO_IMAGE_SYNC_H_
#define STEREO_IMAGE_SYNC_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include "river_ros_util/ros_util.h"
#include <diagnostic_updater/diagnostic_updater.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
using namespace boost::accumulators;

// Standard libs
#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <sstream>
#include <cmath>

namespace stereo_image_sync{

class StereoImageSync
{
private:
  const sensor_msgs::ImageConstPtr nullImagePtr;
  const sensor_msgs::CameraInfoConstPtr nullCameraInfoPtr;

  sensor_msgs::ImageConstPtr last_left_;
  sensor_msgs::CameraInfoConstPtr last_left_info_;
  sensor_msgs::ImageConstPtr last_right_;
  sensor_msgs::CameraInfoConstPtr last_right_info_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  image_transport::CameraPublisher left_pub_;
  image_transport::CameraPublisher right_pub_;

  image_transport::CameraSubscriber left_sub_;
  image_transport::CameraSubscriber right_sub_;

  double max_time_diff_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  diagnostic_updater::Updater updater;
  void update_diagnostic_status(diagnostic_updater::DiagnosticStatusWrapper &stat);

  accumulator_set< double, stats< tag::min, tag::max, tag::mean > > stats_acc;
public:
  StereoImageSync(ros::NodeHandle nh = ros::NodeHandle(), ros::NodeHandle pnh = ros::NodeHandle("~"));

  void process_input();

  void left_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);

  void right_callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);

};

}

#endif

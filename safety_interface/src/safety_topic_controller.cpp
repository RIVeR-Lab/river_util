#include <pluginlib/class_list_macros.h>
#include <safety_interface/safety_interface.h>
#include <safety_interface/SoftwareStop.h>
#include <controller_interface/controller.h>

namespace safety_interface{

class SafetyTopicController: public controller_interface::Controller<safety_interface::SafetyInterface>{

public:
  SafetyTopicController() : state_(safety_state::OK) {}
  ~SafetyTopicController() {sub_stop_.shutdown();}
  bool init(safety_interface::SafetyInterface* hw, ros::NodeHandle &root_nh, ros::NodeHandle &nh) {
    safety_interface_ = hw;
    sub_stop_ = root_nh.subscribe<safety_interface::SoftwareStop>("stop", 1, &SafetyTopicController::stopCB, this);
    return true;
  }
  void starting(const ros::Time& time) {
    state_ = safety_interface_->get_state();
  }
  void update(const ros::Time& time, const ros::Duration& period) {
    safety_interface_->set_state(state_);
  }
  void stopCB(const safety_interface::SoftwareStopConstPtr& msg) {
    if(msg->stop){
      if(state_ != safety_state::PAUSE)
        ROS_WARN_STREAM("Robot paused: " << msg->message);
      state_ = safety_state::PAUSE;
    }
    else{
      if(state_ != safety_state::OK)
        ROS_WARN_STREAM("Robot ok: " << msg->message);
      state_ = safety_state::OK;
    }
  }
private:
  safety_interface::SafetyInterface* safety_interface_;
  ros::Subscriber sub_stop_;
  safety_state::value state_;
};

}

PLUGINLIB_EXPORT_CLASS(safety_interface::SafetyTopicController,controller_interface::ControllerBase)

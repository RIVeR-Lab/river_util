#include <stdlib.h>
#include <device_driver_base/device_driver.h>

namespace device_driver{

DeviceDriver::DeviceDriver(double spin_rate, const ros::NodeHandle& node, const ros::NodeHandle& p_node) : spin_rate_(spin_rate), node_(node), p_node_(p_node), state_(device_driver_state::STOPPED){
  restart_timer_ = node_.createTimer(ros::Duration(0.5), boost::bind(&DeviceDriver::gotoState, this, device_driver_state::RUNNING), true, false);
}

void DeviceDriver::spin(){
	ROS_DEBUG("Driver Spinning");
	gotoState(device_driver_state::RUNNING);
	while(ros::ok()){
		if(getState()==device_driver_state::RUNNING)
			runningLoop();
		ros::spinOnce();
		spin_rate_.sleep();
	}
	gotoState(device_driver_state::STOPPED);
}

void DeviceDriver::runningLoop(){
}

void DeviceDriver::gotoState(device_driver_state_t new_state){
  ROS_DEBUG("gotoState: %d->%d", state_, new_state);
  if(state_==new_state)
    return;
  try{
    if(new_state>state_){
      if(state_<=device_driver_state::RUNNING){
	_leave(device_driver_state::RUNNING);
	state_ = device_driver_state::OPEN;
      }
      if(state_<=device_driver_state::OPEN){
	_leave(device_driver_state::OPEN);
	state_ = device_driver_state::STOPPED;
      }
    }
    else if(new_state<state_){
      if(state_>=device_driver_state::OPEN){
	_enter(device_driver_state::OPEN);
	state_ = device_driver_state::OPEN;
      }
      if(state_>=device_driver_state::RUNNING){
	_enter(device_driver_state::RUNNING);
	state_ = device_driver_state::RUNNING;
      }
    }
    state_ = new_state;
  } catch(Exception& e){
    handleException(e);
  }


}

void DeviceDriver::_leave(device_driver_state_t state){
  ROS_DEBUG("Driver leaving state: %d", state);

  std::pair<std::multimap<device_driver_state_t, DeviceDriverStateActionPtr>::iterator,
	    std::multimap<device_driver_state_t, DeviceDriverStateActionPtr>::iterator> range
    = state_actions_.equal_range(state);
  //iterate backwards
  std::multimap<device_driver_state_t, DeviceDriverStateActionPtr>::iterator itr = range.second;
  while(itr != range.first){
    itr--;
    itr->second->stop();
  }
}
void DeviceDriver::_enter(device_driver_state_t state){
  ROS_DEBUG("Driver entering state: %d", state);

  std::pair<std::multimap<device_driver_state_t, DeviceDriverStateActionPtr>::iterator,
	    std::multimap<device_driver_state_t, DeviceDriverStateActionPtr>::iterator> range
    = state_actions_.equal_range(state);
  std::multimap<device_driver_state_t, DeviceDriverStateActionPtr>::iterator itr = range.first;
  try{
    while(itr!=range.second){
      itr->second->start();
      itr++;
    }
  } catch(Exception& e){
    while(itr!=range.first){//first undo anything that was done
      //stop is not called on the action that threw an exception
      itr--;
      itr->second->stop();
    }
    throw;
  }
}

void DeviceDriver::handleException(Exception& e){
  ROS_ERROR_STREAM("Driver got error: "<<e.what());
  //e.printStackTrace();
  gotoState(device_driver_state::STOPPED);
  restart_timer_.stop();//must stop oneshot timer in order to restart it
  restart_timer_.start();
}


void DeviceDriver::addDriverAction(device_driver_state_t state, DeviceDriverStateActionPtr new_action){
    state_actions_.insert(std::pair<device_driver_state_t, DeviceDriverStateActionPtr>(state, new_action));
}



StateTimerAction::StateTimerAction(DeviceDriver* driver, ros::Duration& period, const ros::TimerCallback &user_callback):user_callback_(user_callback), driver_(driver){
  timer_ = driver_->node_.createTimer(period, boost::bind(&StateTimerAction::_callback, this, _1), false, false);
}
void StateTimerAction::_callback(const ros::TimerEvent& te){
  try{
    user_callback_(te);
  } catch(Exception& e){
    driver_->handleException(e);
  }
}
void StateTimerAction::start(){
  timer_.start();
}
void StateTimerAction::stop() throw(){
  timer_.stop();
}



}

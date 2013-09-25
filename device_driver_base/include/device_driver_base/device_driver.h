#ifndef DEVICE_DRIVER_H_
#define DEVICE_DRIVER_H_

#include <stdlib.h>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/any.hpp>
#include <ros/ros.h>
#include <device_driver_base/driver_util.h>

namespace device_driver{

class DeviceDriver;

namespace device_driver_state{
  typedef enum{
    STOPPED = 3,
    OPEN = 1,
    RUNNING = 0
  } device_driver_state_t;
}
typedef device_driver_state::device_driver_state_t device_driver_state_t;

class DeviceDriverStateAction{
 public:
  /**
   * Triggered when a state is entered. If this throws a driver exception then
   * this method should reverse any actions it took (stop will not be called)
   */
  virtual void start() = 0;
  /**
   * Triggered when a state is exited
   * This method should not throw an exception
   */
  virtual void stop() throw() = 0;
  virtual ~DeviceDriverStateAction(){}
};
typedef boost::shared_ptr<DeviceDriverStateAction> DeviceDriverStateActionPtr;


class StateTimerAction : public DeviceDriverStateAction{
protected:
  ros::Timer timer_;
private:
  ros::TimerCallback user_callback_;
  DeviceDriver* driver_;
  void _callback(const ros::TimerEvent& e);
 public:
  StateTimerAction(DeviceDriver* driver, ros::Duration& period, const ros::TimerCallback &user_callback);
  virtual void start();
  virtual void stop() throw();
};


class StateFunctionsAction : public DeviceDriverStateAction{
 private:
  boost::function<void()> start_function_;
  boost::function<void() throw()> stop_function_;
 public:
  StateFunctionsAction(boost::function<void()> start_function, boost::function<void() throw()> stop_function) : start_function_(start_function), stop_function_(stop_function){}
  virtual void start(){start_function_();}
  virtual void stop() throw(){stop_function_();}
};

class DeviceDriver{
 private:
  ros::Rate spin_rate_;
 public:
  ros::NodeHandle node_;
  ros::NodeHandle p_node_;
 private:
  device_driver_state_t state_;
  std::multimap<device_driver_state_t, DeviceDriverStateActionPtr> state_actions_;
  ros::Timer restart_timer_;
 private:
  void _leave(device_driver_state_t state);
  void _enter(device_driver_state_t state);

 protected:
  void gotoState(device_driver_state_t new_state);
  void addDriverAction(device_driver_state_t state, DeviceDriverStateActionPtr new_action_);

  template<class T> void addDriverTimer(device_driver_state_t state, ros::Duration period, void(T::*callback)(const ros::TimerEvent &) const, T *obj){
    ros::Timer timer = node_.createTimer(period, callback, obj, false, false);
    addDriverAction(state, DeviceDriverStateActionPtr(new StateTimerAction(this, period, boost::bind(callback, obj, _1))));
  }
  template<class T> void addDriverTimer(device_driver_state_t state, ros::Duration period, void(T::*callback)(const ros::TimerEvent &), T *obj){
    ros::Timer timer = node_.createTimer(period, callback, obj, false, false);
    addDriverAction(state, DeviceDriverStateActionPtr(new StateTimerAction(this, period, boost::bind(callback, obj, _1))));
  }

  template<class T> void addDriverStateFunctions(device_driver_state_t state, void(T::*start_callback)(), void(T::*stop_callback)(), T *obj){
    addDriverAction(state, DeviceDriverStateActionPtr(new StateFunctionsAction(boost::bind(start_callback, obj), boost::bind(stop_callback, obj))));
  }


 public:
  DeviceDriver(double spin_rate = 100, const ros::NodeHandle& node = ros::NodeHandle(), const ros::NodeHandle& p_node = ros::NodeHandle("~"));
  virtual void spin();
  virtual void runningLoop();
  virtual ~DeviceDriver(){}

  device_driver_state_t getState(){return state_;}
  void handleException(Exception& e);

  friend class StateTimerAction;
};

}

#endif //DEVICE_DRIVER_H_

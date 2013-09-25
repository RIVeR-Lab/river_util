#ifndef RECONFIGURABLE_DEVICE_DRIVER_H_
#define RECONFIGURABLE_DEVICE_DRIVER_H_

#include <stdlib.h>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/any.hpp>
#include <ros/ros.h>
#include <device_driver_base/device_driver.h>
#include <device_driver_base/SensorLevels.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>

namespace device_driver{




class ReconfigurableTimer : public StateTimerAction{
 public:
  ReconfigurableTimer(DeviceDriver* driver, ros::Duration& period, const ros::TimerCallback &callback):StateTimerAction(driver, period, callback){
  }
  void setPeriod(ros::Duration period){
	  timer_.setPeriod(period);
  }
};
typedef boost::shared_ptr<ReconfigurableTimer> ReconfigurableTimerPtr;











class AbstractReconfigurableSubscriber : public DeviceDriverStateAction{
public:
	virtual void setTopic(std::string& topic) = 0;
	virtual void setQueueSize(uint32_t queue_size) = 0;
};
template <class M> class ReconfigurableSubscriber : public AbstractReconfigurableSubscriber{
protected:
  ros::Subscriber subscriber_;
private:
  DeviceDriver* driver_;
  std::string topic_;
  uint32_t queue_size_;
  const boost::function< void(const boost::shared_ptr< M const > &)> user_callback_;

  void _subscriber_callback(const boost::shared_ptr< M const > & v){
	  try{
		  user_callback_(v);
	  } catch(Exception& e){
		  driver_->handleException(e);
	  }
  }
 public:
  ReconfigurableSubscriber(DeviceDriver* driver, const std::string &topic, uint32_t queue_size,
		  const boost::function< void(const boost::shared_ptr< M const > &)> &user_callback) :
		  driver_(driver), topic_(topic), queue_size_(queue_size), user_callback_(user_callback){
  }
  virtual void start(){
	  subscriber_ = driver_->node_.subscribe<M>(topic_, queue_size_, boost::bind(&ReconfigurableSubscriber::_subscriber_callback, this, _1));
  }
  virtual void stop() throw(){
	  subscriber_.shutdown();
  }
  virtual void setTopic(std::string& topic){
	  topic_ = topic;
  }
  virtual void setQueueSize(uint32_t queue_size){
	  queue_size_ = queue_size;
  }
};
typedef boost::shared_ptr<AbstractReconfigurableSubscriber> ReconfigurableSubscriberPtr;










class AbstractReconfigurableThrottledSubscriber : public AbstractReconfigurableSubscriber{
public:
	virtual void setMaxRate(double max_frequency) = 0;
};
template <class M> class ReconfigurableThrottledSubscriber : public AbstractReconfigurableThrottledSubscriber{
protected:
  ros::Timer timer_;
  ros::Subscriber subscriber_;
private:
  DeviceDriver* driver_;
  std::string topic_;
  uint32_t queue_size_;
  const boost::function< void(const boost::shared_ptr< M const > &)> user_callback_;

  const boost::shared_ptr< M const > nullPtr_;
  boost::shared_ptr< M const > latest_;
  void _timer_callback(const ros::TimerEvent& te){
	  if(latest_){
		  try{
			  user_callback_(latest_);
		  } catch(Exception& e){
			  driver_->handleException(e);
		  }
		  latest_ = nullPtr_;
	  }
  }
  void _subscriber_callback(const boost::shared_ptr< M const > & v){
	  latest_ = v;
  }
 public:
  ReconfigurableThrottledSubscriber(DeviceDriver* driver, double max_frequency, const std::string &topic, uint32_t queue_size,
		  const boost::function< void(const boost::shared_ptr< M const > &)> &user_callback) :
		  driver_(driver), topic_(topic), queue_size_(queue_size), user_callback_(user_callback){
	  timer_ = driver_->node_.createTimer(ros::Duration(1/max_frequency), boost::bind(&ReconfigurableThrottledSubscriber::_timer_callback, this, _1), false, false);
  }
  virtual void start(){
	  timer_.start();
	  subscriber_ = driver_->node_.subscribe<M>(topic_, queue_size_, boost::bind(&ReconfigurableThrottledSubscriber::_subscriber_callback, this, _1));
  }
  virtual void stop() throw(){
	  subscriber_.shutdown();
	  timer_.stop();
	  latest_ = nullPtr_;
  }
  virtual void setMaxRate(double max_frequency){
	  timer_.setPeriod(ros::Duration(1/max_frequency));
  }
  virtual void setTopic(std::string& topic){
	  topic_ = topic;
  }
  virtual void setQueueSize(uint32_t queue_size){
	  queue_size_ = queue_size;
  }
};
typedef boost::shared_ptr<AbstractReconfigurableThrottledSubscriber> ReconfigurableThrottledSubscriberPtr;









class AbstractReconfigurableAdvertise : public DeviceDriverStateAction{
protected:
	ros::Publisher publisher_;
public:
	virtual ~AbstractReconfigurableAdvertise(){};
	virtual void setTopic(std::string& topic) = 0;
	virtual void setQueueSize(uint32_t queue_size) = 0;
	virtual void setLatch(bool latch) = 0;

	template<typename M> void publish(const M &message){
		publisher_.publish(message);
	}
};
template <class M> class ReconfigurableAdvertise : public AbstractReconfigurableAdvertise{
private:
	ros::NodeHandle nh_;
	std::string topic_;
	uint32_t queue_size_;
	bool latch_;
public:
	ReconfigurableAdvertise(ros::NodeHandle nh, const std::string &topic, uint32_t queue_size, bool latch=false):
		nh_(nh), topic_(topic), queue_size_(queue_size), latch_(latch){}
	virtual void start(){
		publisher_ = nh_.advertise<M>(topic_, queue_size_, latch_);
	}
	virtual void stop() throw(){
		publisher_.shutdown();
	}

	virtual void setTopic(std::string& topic){
		topic_ = topic;
	}
	virtual void setQueueSize(uint32_t queue_size){
		queue_size_ = queue_size;
	}
	virtual void setLatch(bool latch){
		latch_ = latch;
	}
};
typedef boost::shared_ptr<AbstractReconfigurableAdvertise> ReconfigurableAdvertisePtr;






class AbstractReconfigurableActionServer : public DeviceDriverStateAction{
public:
	virtual void setName(std::string& name) = 0;
};
template <class A> class ReconfigurableActionServer : public AbstractReconfigurableActionServer{
protected:
  boost::shared_ptr<actionlib::SimpleActionServer<A> > as_;
private:
  DeviceDriver* driver_;
  std::string name_;
  const boost::function<void(const typename A::_action_goal_type::_goal_type::ConstPtr &,
		  boost::shared_ptr<actionlib::SimpleActionServer<A> >&)> user_callback_;

  void _execute_callback(const typename A::_action_goal_type::_goal_type::ConstPtr & goal){
	  try{
		  user_callback_(goal, as_);
	  } catch(Exception& e){
		  //TODO actually handle exception in main thread
		  //driver_->handleException(e);
	  }
  }

 public:
  ReconfigurableActionServer(DeviceDriver* driver, const std::string &name,
		  const boost::function<void(const typename A::_action_goal_type::_goal_type::ConstPtr &,
		  		  boost::shared_ptr<actionlib::SimpleActionServer<A> >&)> user_callback) :
		  driver_(driver), name_(name), user_callback_(user_callback){
  }
  virtual void start(){
	  as_ = boost::shared_ptr<actionlib::SimpleActionServer<A> >(
			  new actionlib::SimpleActionServer<A>(
					  driver_->node_, name_, boost::bind(&ReconfigurableActionServer::_execute_callback, this, _1), false));
	  as_->start();
  }
  virtual void stop() throw(){
	  as_->shutdown();
	  as_ = boost::shared_ptr<actionlib::SimpleActionServer<A> >();//set to null
  }
  virtual void setName(std::string& name){
	  name_ = name;
  }
};
typedef boost::shared_ptr<AbstractReconfigurableActionServer> ReconfigurableActionServerPtr;








template<class T> class ReconfigurableDeviceDriver : public DeviceDriver{
 private:
  dynamic_reconfigure::Server<T> dynamic_reconfigure_server_;
 private:
  void callback(T &config, uint32_t level){
    device_driver_state_t old_state = getState();

    device_driver_state_t reconfigure_state = device_driver_state::STOPPED;
    if(level==device_driver_base::SensorLevels::RECONFIGURE_OPEN)
    	reconfigure_state = device_driver_state::OPEN;
    else if(level==device_driver_base::SensorLevels::RECONFIGURE_RUNNING)
    	reconfigure_state = device_driver_state::RUNNING;

	ROS_DEBUG("Dynamic Reconfigure Updated while in %d (switching into %d if needed)", old_state, reconfigure_state);

    if(getState()<reconfigure_state)//do we actually need to change states
        gotoState(reconfigure_state);

    try{
		if(reconfigure_state>=device_driver_state::STOPPED)
			reconfigureStopped(config);
		if(reconfigure_state>=device_driver_state::OPEN)
			reconfigureOpen(config);
		if(reconfigure_state>=device_driver_state::RUNNING)
			reconfigureRunning(config);
		ROS_DEBUG("Reconfigure complete, returning to %d", old_state);
	    gotoState(old_state);
    } catch(Exception& e){
    	ROS_ERROR_STREAM("Driver got error during reconfigure: "<<e.what());
        handleException(e);
    }
  }
 protected:
  virtual void reconfigureStopped(T &config){}
  virtual void reconfigureOpen(T &config){}
  virtual void reconfigureRunning(T &config){}
 protected:
  template <class M> ReconfigurableAdvertisePtr addReconfigurableAdvertise(device_driver_state_t state, const std::string &topic, uint32_t queue_size, bool latch=false){
	  ReconfigurableAdvertisePtr ptr(new ReconfigurableAdvertise<M>(node_, topic, queue_size, latch));
	  addDriverAction(state, ptr);
	  return ptr;
  }
  template <class M> ReconfigurableSubscriberPtr addReconfigurableSubscriber(device_driver_state_t state, const std::string &topic, uint32_t queue_size,
		  const boost::function< void(const boost::shared_ptr< M const > &)> &user_callback){
	  ReconfigurableSubscriberPtr ptr(new ReconfigurableSubscriber<M>(this, topic, queue_size, user_callback));
	  addDriverAction(state, ptr);
	  return ptr;
  }
  template <class M> ReconfigurableThrottledSubscriberPtr addReconfigurableThrottledSubscriber(device_driver_state_t state, double max_frequency, const std::string &topic, uint32_t queue_size,
		  const boost::function< void(const boost::shared_ptr< M const > &)> &user_callback){
	  ReconfigurableThrottledSubscriberPtr ptr(new ReconfigurableThrottledSubscriber<M>(this, max_frequency, topic, queue_size, user_callback));
	  addDriverAction(state, ptr);
	  return ptr;
  }
  template<class O> ReconfigurableTimerPtr addReconfigurableTimer(device_driver_state_t state, ros::Duration period, void(O::*callback)(const ros::TimerEvent &) const, O *obj){
    ros::Timer timer = node_.createTimer(period, callback, obj, false, false);
    ReconfigurableTimerPtr ptr(new ReconfigurableTimer(this, period, boost::bind(callback, obj, _1)));
    addDriverAction(state, ptr);
    return ptr;
  }
  template<class O> ReconfigurableTimerPtr addReconfigurableTimer(device_driver_state_t state, ros::Duration period, void(O::*callback)(const ros::TimerEvent &), O *obj){
    ros::Timer timer = node_.createTimer(period, callback, obj, false, false);
    ReconfigurableTimerPtr ptr(new ReconfigurableTimer(this, period, boost::bind(callback, obj, _1)));
    addDriverAction(state, ptr);
    return ptr;
  }
  template <class A> ReconfigurableActionServerPtr addReconfigurableActionServer(device_driver_state_t state, const std::string &name,
		  const boost::function<void(const typename A::_action_goal_type::_goal_type::ConstPtr &,
		  		  boost::shared_ptr<actionlib::SimpleActionServer<A> >&)> &user_callback){
	  ReconfigurableActionServerPtr ptr(new ReconfigurableActionServer<A>(this, name, user_callback));
	  addDriverAction(state, ptr);
	  return ptr;
  }
 public:
  ReconfigurableDeviceDriver(double spin_rate = 100, const ros::NodeHandle& node = ros::NodeHandle(), const ros::NodeHandle& p_node = ros::NodeHandle("~")) :
	  DeviceDriver(spin_rate, node, p_node), dynamic_reconfigure_server_(p_node_){
  }
  virtual void spin(){
	  dynamic_reconfigure_server_.setCallback(boost::bind(&ReconfigurableDeviceDriver::callback, this, _1, _2));//must delay this or else child class methods will not have been installed yet
	  DeviceDriver::spin();
  }
  virtual ~ReconfigurableDeviceDriver(){}
};

}

#endif //RECONFIGURABLE_DEVICE_DRIVER_H_

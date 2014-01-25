#ifndef ROS_UTIL_H_
#define ROS_UTIL_H_

#include <stdlib.h>
#include <cstddef>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <ros/ros.h>
#include <execinfo.h>

namespace river_ros_util{

/**
 * Macro to throw an exception like prinf and extract debugging information
 */
#define THROWF(except, msg, ...) do{					\
		char buf[1000];						\
		snprintf(buf, 1000, msg " (%s, line %d, in function %s)" , ##__VA_ARGS__, __FILE__, __LINE__, __FUNCTION__); \
		throw except(buf);					\
  }while(0)


/**
 * A general exception that records the stack when it's thrown
 */
class StackTraceException : public std::runtime_error{
  private:
    void* frames[20];
    size_t frame_size;
 public:
    StackTraceException(const std::string& msg):std::runtime_error(msg){
      frame_size = backtrace(frames, sizeof frames/sizeof(void*));
    }
    void printStackTrace(){
      backtrace_symbols_fd(frames, frame_size, 2);
    }
};




/**
 * @author Mitchell Wills
 * @brief gets a ros parameter and stores it to a variable, will do ROS_WARN if the parameter is not set
 * @param n the node to get the parameter from
 * @param var the variable to store the parameter value into
 * @param param_name the name of the parameter
 * @return true if the parameter value was set
 */
template<class T> static inline bool get_param(ros::NodeHandle& n, T& var, std::string param_name){
  if(!n.getParam(param_name, var)){
    ROS_WARN_STREAM("Parameter <"<<param_name<<"> not set. Using default value '"<<var<<"'");
    return false;
  }
  return true;
}

/**
 * @author Mitchell Wills
 * @brief defines a variable of a given type and stores the value of the ROS parameter
 * @param type the type of the variable
 * @param var_name the name of the variable that will be defined
 * @param param_name the name of the parameter to load into the variable
 * @param default_value the default value to store in the variable fit the parameter is not set
 */
#define define_and_get_global_param_(type, var_name, param_name, default_value) \
	type var_name(default_value);					\
	::river_ros_util::get_global_param(var_name, param_name)

/**
 * @author Mitchell Wills
 * @brief defines a variable of a given type and stores the value of the ROS parameter
 * @param n the node to get the parameter from
 * @param type the type of the variable
 * @param var_name the name of the variable that will be defined
 * @param param_name the name of the parameter to load into the variable
 * @param default_value the default value to store in the variable fit the parameter is not set
 */
#define define_and_get_param(n, type, var_name, param_name, default_value) \
	type var_name(default_value);					\
	::river_ros_util::get_param(n, var_name, param_name)


/**
 * @author Mitchell Wills
 * @brief gets a ros parameter and stores it to a variable, will do ROS_WARN if the parameter is not set
 * @param var the variable to store the parameter value into
 * @param param_name the name of the parameter
 * @return true if the parameter value was set
 */
template<class T> static inline bool get_global_param(T& var, std::string param_name){
  if(!ros::param::get(param_name, var)){
    ROS_WARN_STREAM("Parameter <"<<param_name<<"> not set. Using default value '"<<var<<"'");
    return false;
  }
  return true;
}

}

#endif//ROS_UTIL

#ifndef DRIVER_UTIL_H_
#define DRIVER_UTIL_H_

#include <stdlib.h>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <ros/ros.h>
#include <execinfo.h>

namespace device_driver{

/**
 * Macro to throw an exception like prinf and extract debugging information
 */
#define DRIVER_EXCEPT(except, msg, ...) do{					\
		char buf[1000];						\
		snprintf(buf, 1000, msg " (%s, line %d, in function %s)" , ##__VA_ARGS__, __FILE__, __LINE__, __FUNCTION__); \
		throw except(buf);					\
  }while(0)


/**
 * A general exception which is the parent class of all exceptions thrown by the library
 */
class Exception : public std::runtime_error{
  private:
    void* frames[20];
    size_t frame_size;
 public:
  Exception(const std::string& msg):std::runtime_error(msg){
      frame_size = backtrace(frames, sizeof frames/sizeof(void*));
    }
    void printStackTrace(){
      backtrace_symbols_fd(frames, frame_size, 2);
    }
};
/**
 * @author Mitchell Wills
 * @brief An exception indicating that an invalid argument was provided to a function
 */
class IllegalArgumentException : public Exception{
 public:
  IllegalArgumentException(const std::string& msg):Exception(msg){}

};

class DeviceNotOpenException : public Exception{
 public:
  DeviceNotOpenException(const std::string msg):Exception(msg){}
};



/**
 * @author Mitchell Wills
 * @brief gets a ros parameter and stores it to a variable, will do ROS_WARN if the parameter is not set
 * @param var the variable to store the parameter value into
 * @param param_name the name of the parameter
 * @return true if the parameter value was set
 */
template<class T> static inline bool get_param(T& var, std::string param_name){
  if(!ros::param::get(param_name, var)){
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
#define define_and_get_param(type, var_name, param_name, default_value) \
	type var_name(default_value);					\
	::device_driver::get_param(var_name, param_name)

}

/**
 * @author Mitchell Wills
 * @brief returns true if the actual value matches the expected
 * @param expected the expected value
 * @param actual the value to be compared to the expected
 */
static inline bool streq(const char* expected, const char* actual){
  return !strcmp(expected, actual);
}


#endif//DRIVER_UTIL

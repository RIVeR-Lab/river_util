#ifndef DRIVER_UTIL_H_
#define DRIVER_UTIL_H_

#include <stdlib.h>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <ros/ros.h>
#include <execinfo.h>
#include <river_ros_util/ros_util.h>

namespace device_driver{

/**
 * Macro to throw an exception like prinf and extract debugging information
 */
#define DRIVER_EXCEPT(except, msg, ...) THROWF(except, msg, ##__VA_ARGS__)


/**
 * A general exception which is the parent class of all exceptions thrown by the library
 */
class Exception : public river_ros_util::StackTraceException{
 public:
  Exception(const std::string& msg):river_ros_util::StackTraceException(msg){}
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

}


#endif//DRIVER_UTIL

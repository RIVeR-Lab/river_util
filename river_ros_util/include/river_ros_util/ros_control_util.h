#ifndef ROS_CONTROL_UTIL_H_
#define ROS_CONTROL_UTIL_H_

#include <stdlib.h>
#include <stdint.h>
#include <ros/ros.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

namespace river_ros_util{


static inline transmission_interface::ActuatorData create_transmission_actuator_data(double* pos, double* vel, double* eff){
  transmission_interface::ActuatorData a_state_data;
  if(pos)
    a_state_data.position.push_back(pos);
  if(vel)
    a_state_data.velocity.push_back(vel);
  if(eff)
    a_state_data.effort.push_back(eff);
  return a_state_data;
}
static inline transmission_interface::JointData create_transmission_joint_data(double* pos, double* vel, double* eff){
  transmission_interface::JointData j_state_data;
  if(pos)
    j_state_data.position.push_back(pos);
  if(vel)
    j_state_data.velocity.push_back(vel);
  if(eff)
    j_state_data.effort.push_back(eff);
  return j_state_data;
}
static inline transmission_interface::ActuatorData create_transmission_actuator_state_data(hardware_interface::ActuatorStateHandle& actuator_handle){
  return create_transmission_actuator_data((double*)actuator_handle.getPositionPtr(),
                              (double*)actuator_handle.getVelocityPtr(),
                              (double*)actuator_handle.getEffortPtr());
}



class RobotHWComponent{
 public:
  virtual void read() = 0;
  virtual void write() = 0;
};
typedef boost::shared_ptr<RobotHWComponent> RobotHWComponentPtr;

class JointData{
 public:
  JointData():pos(0), vel(0), eff(0), cmd(0){}
  double pos;
  double vel;
  double eff;
  double cmd;
  transmission_interface::JointData transmission_state_data(){
    return create_transmission_joint_data(&pos, &vel, &eff);
  }
  hardware_interface::JointStateHandle state_handle(std::string joint_name){
    return hardware_interface::JointStateHandle(joint_name, &pos, &vel, &eff);
  }
};
typedef boost::shared_ptr<JointData> JointDataPtr;
typedef boost::shared_ptr<transmission_interface::Transmission> TransmissionPtr;


}


#endif//ROS_CONTROL_UTIL

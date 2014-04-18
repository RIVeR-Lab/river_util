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





class AbstractRobotHW : public hardware_interface::RobotHW
{
 protected:
  void build_transmissions(std::vector<transmission_interface::TransmissionInfo> transmissions){
    BOOST_FOREACH(transmission_interface::TransmissionInfo info, transmissions){
      build_transmission(info);
    }
  }
  void build_transmission(transmission_interface::TransmissionInfo info){
    if(info.joints_.size()!=1){
      ROS_ERROR_STREAM("Transmission '"<<info.name_<<"' does not have exactly one joint");
      return;
    }
    if(info.actuators_.size()!=1){
      ROS_ERROR_STREAM("Transmission '"<<info.name_<<"' does not have exactly one actuator");
      return;
    }
    const std::string& transmission_name = info.name_;
    const std::string& transmission_type = info.type_;
    const std::string& joint_name = info.joints_[0].name_;
    const std::string& hardware_interface = info.actuators_[0].hardware_interface_;
    const std::string& actuator_name = info.actuators_[0].name_;
    JointDataPtr joint_data = JointDataPtr(new JointData());
    js_interface.registerHandle(joint_data->state_handle(joint_name));

    hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(js_interface.getHandle(joint_name),
                                                                                   &joint_data->cmd);

    TransmissionPtr transmission;
    if(transmission_type == "transmission_interface/SimpleTransmission"){
      double offset = 0;
      double reduction = 1;//can't actually get reduction
      transmission = TransmissionPtr(new transmission_interface::SimpleTransmission(reduction, offset));
    }
    else{
      ROS_FATAL_STREAM_NAMED("ros_control_util","No matching transmission found for '"
			       << transmission_type );
      return;
    }

    hardware_interface::ActuatorStateHandle actuator_handle = as_interface.getHandle(actuator_name);
    atojs_interface.registerHandle(transmission_interface::ActuatorToJointStateHandle(actuator_name,
                                                               &*transmission,
							       create_transmission_actuator_state_data(actuator_handle),
							       joint_data->transmission_state_data()));
    transmission_interface::ActuatorData a_cmd_data;
    transmission_interface::JointData j_cmd_data;
    if(hardware_interface == "EffortJointInterface"){
      je_interface.registerHandle(joint_handle);

      hardware_interface::ActuatorHandle actuator_cmd_handle = ae_interface.getHandle(actuator_name);
      j_cmd_data.effort.push_back(&joint_data->cmd);
      a_cmd_data.effort.push_back((double*)actuator_cmd_handle.getCommandPtr());
      jtoae_interface.registerHandle(transmission_interface::JointToActuatorEffortHandle(actuator_name,
								 &*transmission,
								 a_cmd_data,
								 j_cmd_data));
    }
    else if(hardware_interface == "PositionJointInterface"){
      jp_interface.registerHandle(joint_handle);

      hardware_interface::ActuatorHandle actuator_cmd_handle = ap_interface.getHandle(actuator_name);
      j_cmd_data.position.push_back(&joint_data->cmd);
      a_cmd_data.position.push_back((double*)actuator_cmd_handle.getCommandPtr());
      jtoap_interface.registerHandle(transmission_interface::JointToActuatorPositionHandle(actuator_name,
								 &*transmission,
								 a_cmd_data,
								 j_cmd_data));
    }
    else if(hardware_interface == "VelocityJointInterface"){
      jv_interface.registerHandle(joint_handle);

      hardware_interface::ActuatorHandle actuator_cmd_handle = av_interface.getHandle(actuator_name);
      j_cmd_data.velocity.push_back(&joint_data->cmd);
      a_cmd_data.velocity.push_back((double*)actuator_cmd_handle.getCommandPtr());
      jtoav_interface.registerHandle(transmission_interface::JointToActuatorVelocityHandle(actuator_name,
								 &*transmission,
								 a_cmd_data,
								 j_cmd_data));
    }
    else{
      ROS_FATAL_STREAM_NAMED("ros_control_util","No matching hardware interface found for '"
			       << hardware_interface );
      return;
    }


    joint_datas.push_back(joint_data);
    transmissions.push_back(transmission);
  }

  void add_actuator(RobotHWComponentPtr component){
    actuators.push_back(component);
  }

  void register_interfaces(){
    registerInterface(&as_interface);
    registerInterface(&ap_interface);
    registerInterface(&av_interface);
    registerInterface(&ae_interface);

    registerInterface(&js_interface);
    registerInterface(&jp_interface);
    registerInterface(&jv_interface);
    registerInterface(&je_interface);

    registerInterface(&atojs_interface);
    registerInterface(&jtoap_interface);
    registerInterface(&jtoav_interface);
    registerInterface(&jtoae_interface);
  }

 public:
  void write(){
    for(unsigned int i = 0; i<actuators.size(); ++i){
      actuators[i]->write();
    }
    atojs_interface.propagate();
  }
  void read(){
    jtoap_interface.propagate();
    jtoav_interface.propagate();
    jtoae_interface.propagate();
    for(unsigned int i = 0; i<actuators.size(); ++i){
      actuators[i]->read();
    }
  }

 protected:
  hardware_interface::ActuatorStateInterface as_interface;
  hardware_interface::PositionActuatorInterface ap_interface;
  hardware_interface::VelocityActuatorInterface av_interface;
  hardware_interface::EffortActuatorInterface ae_interface;
  hardware_interface::JointStateInterface js_interface;
  hardware_interface::PositionJointInterface jp_interface;
  hardware_interface::VelocityJointInterface jv_interface;
  hardware_interface::EffortJointInterface je_interface;
  transmission_interface::ActuatorToJointStateInterface atojs_interface;
  transmission_interface::JointToActuatorPositionInterface jtoap_interface;
  transmission_interface::JointToActuatorVelocityInterface jtoav_interface;
  transmission_interface::JointToActuatorEffortInterface jtoae_interface;

 private:
  std::vector<RobotHWComponentPtr> actuators;
  std::vector<JointDataPtr> joint_datas;
  std::vector<TransmissionPtr> transmissions;
};


}


#endif//ROS_CONTROL_UTIL

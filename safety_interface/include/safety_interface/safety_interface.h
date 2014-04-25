#ifndef SAFETY_INTERFACE_SAFETY_INTERFACE_H
#define SAFETY_INTERFACE_SAFETY_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace safety_interface{

namespace safety_state{
  typedef enum {OK, PAUSE, ESTOP} value;
}

class SafetyHandle
{
 public:
};

class SafetyInterface : public hardware_interface::HardwareResourceManager<SafetyHandle> {
 public:
  SafetyInterface(): state(safety_state::OK){}
  safety_state::value get_state(){
    return state;
  }
  void set_state(safety_state::value new_state){
    state = new_state;
  }
 private:;
  safety_state::value state;
};

}


#endif

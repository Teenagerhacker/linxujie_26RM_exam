#include "ins_comm.hpp"

hello_world::remote_control::DT7 *GetRemoteControl() {
  static hello_world::remote_control::DT7 remote_control;
  return &remote_control;
}


#include "ins_robot.hpp"
#include "gimbal.hpp"
#include "robot.hpp"

robot::Robot *GetRobot() { static robot::Robot robot; return &robot; }
robot::Gimbal *GetGimbal() { static robot::Gimbal gimbal; return &gimbal; }


#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
for relative in inc/imu.hpp inc/pid.hpp inc/motor_base.hpp inc/motor_DaMiao.hpp \
                src/motor_DaMiao.cpp src/motor_base.cpp src/mahony.cpp src/ahrs_base.cpp; do
  cmp "Chassis/$relative" "Gimbal/$relative"
done
test_dir=$(mktemp -d /tmp/exam1-tests.XXXXXX)
trap 'rm -rf -- "$test_dir"' EXIT
flags=(-std=c++20 -O1 -g -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-parameter
       -DROBOT_HOST_TEST
       -fsanitize=address,undefined -fno-omit-frame-pointer -include tests/platform.hpp
       -Itests/shims -IChassis/inc -IGimbal/inc -IRobotComponents/component
       -IChassis/RobotModules/component -IGimbal/RobotModules/component)
g++ "${flags[@]}" tests/control_test.cpp RobotComponents/component/gimbal_chassis_comm.cpp \
  Chassis/RobotModules/component/chassis.cpp Gimbal/RobotModules/component/gimbal.cpp \
  Chassis/src/motor_base.cpp Chassis/src/motor_M3508.cpp Gimbal/src/motor_DaMiao.cpp \
  Gimbal/src/DT7.cpp -o "$test_dir/control_test"
"$test_dir/control_test"
g++ "${flags[@]}" tests/imu_test.cpp Chassis/src/mahony.cpp Chassis/src/ahrs_base.cpp \
  -o "$test_dir/imu_test"
"$test_dir/imu_test"

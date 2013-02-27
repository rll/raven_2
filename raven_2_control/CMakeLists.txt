cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_definitions("-DRAVEN_DATA_DIR=${CMAKE_SOURCE_DIR}/data")

if (CMAKE_COMPILER_IS_GNUCXX)
    # message ("CMAKE_COMPILER_IS_GNUCXX")
    #set(CMAKE_CXX_FLAGS_INIT "${CMAKE_CXX_FLAGS_INIT} -Wall -ansi -pedantic")
endif()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(src/raven)
include_directories(include)
include_directories(include/raven)
#include_directories(include/raven/state)


#set some compiler flags
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-result -Wno-missing-field-initializers")

if (CMAKE_COMPILER_IS_GNUCXX)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()


#find_package(OpenRAVE REQUIRED)
#include_directories(${OpenRAVE_INCLUDE_DIRS})
#link_directories(${OpenRAVE_LIBRARY_DIRS})

rosbuild_add_boost_directories()

rosbuild_add_library(r2_utils
src/raven/log.cpp
src/raven/util/timing.cpp
)

rosbuild_add_library(r2_state 
src/raven/state/runlevel.cpp
src/raven/state/dof.cpp
src/raven/state/arm.cpp
src/raven/kinematics/kinematics.cpp
src/raven/state/device.cpp
)
target_link_libraries(r2_state r2_utils)


rosbuild_add_library(r2_controllers 
src/raven/control/controller.cpp
src/raven/control/control_input.cpp

src/raven/control/input/motor_input.cpp
src/raven/control/input/joint_input.cpp
src/raven/control/input/end_effector_grasp_input.cpp
src/raven/control/input/end_effector_pose_input.cpp

src/raven/control/controllers/motor_position_pid.cpp
src/raven/control/controllers/joint_velocity_pi.cpp
src/raven/control/controllers/end_effector_control.cpp
)
target_link_libraries(r2_controllers r2_state r2_utils)

rosbuild_add_executable(r2_control

#src/raven/util/logging.cpp
#src/raven/log.cpp
#src/raven/util/timing.cpp

#src/raven/state/runlevel.cpp
#src/raven/state/dof.cpp
#src/raven/state/arm.cpp
#src/raven/state/device.cpp
#src/raven/kinematics/kinematics.cpp

#src/raven/control/controller.cpp
#src/raven/control/control_input.cpp

#src/raven/control/input/motor_input.cpp
#src/raven/control/input/joint_input.cpp
#src/raven/control/input/end_effector_grasp_input.cpp
#src/raven/control/input/end_effector_pose_input.cpp

#src/raven/control/controllers/motor_position_pid.cpp
#src/raven/control/controllers/joint_velocity_pi.cpp
#src/raven/control/controllers/end_effector_control.cpp

src/raven/state/initializer.cpp
src/raven/state/motor_filters/lpf.cpp

#src/raven/asinw.cpp
src/raven/dof.cpp
src/raven/fwd_cable_coupling.cpp
src/raven/fwd_kinematics.cpp
src/raven/get_USB_packet.cpp
src/raven/globals.cpp
#src/raven/grasp.cpp
src/raven/grav_comp.cpp
src/raven/hmatrix.cpp
src/raven/init.cpp
src/raven/inv_cable_coupling.cpp
src/raven/inv_kinematics.cpp
#src/raven/linear_parabolic_traj.cpp
src/raven/local_io.cpp
src/raven/mapping.cpp
src/raven/shared_modes.cpp
src/raven/motor.cpp
src/raven/network_layer.cpp
src/raven/overdrive_detect.cpp
src/raven/parallel.cpp
src/raven/pid_control.cpp
src/raven/put_USB_packet.cpp
src/raven/ros_io.cpp
src/raven/rt_process_preempt.cpp
src/raven/rt_raven.cpp
src/raven/state_estimate.cpp
src/raven/state_machine.cpp
src/raven/homing.cpp
src/raven/trajectory.cpp
src/raven/console_process.cpp
src/raven/control_process.cpp
#src/raven/trajectory_gen_ee449.cpp
src/raven/t_to_DAC_val.cpp
src/raven/update_atmel_io.cpp
src/raven/update_device_state.cpp
src/raven/USB_init.cpp
src/raven/utils.cpp
#src/raven/velocity.cpp
src/raven/saveload.cpp
)

rosbuild_link_boost(r2_control filesystem system)

target_link_libraries(r2_control r2_state r2_controllers r2_utils)

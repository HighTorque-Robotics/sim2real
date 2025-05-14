cmake_minimum_required(VERSION 3.0.2)
project(sim2real)
set(CMAKE_CXX_STANDARD 17)


get_filename_component(WORKSPACE_DIR_FROM_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} DIRECTORY)
get_filename_component(PARENT_DIR ${WORKSPACE_DIR_FROM_DEVEL_PREFIX} DIRECTORY)
get_filename_component(GRANDPARENT_DIR ${PARENT_DIR} DIRECTORY)
set(PythonInterp_DIR /usr/bin/python3)
set(RKNN_PATH ${GRANDPARENT_DIR}/rknn)
add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(STATUS "Building for ARM (aarch64)")
    add_definitions(-DPLATFORM_ARM)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    message(STATUS "Building for x86_64")
    add_definitions(-DPLATFORM_X86_64)
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

# Find catkin
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    find_package(OpenVINO REQUIRED)
    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
endif()
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    livelybot_msg
    serial
    # dynamic_reconfigure
    livelybot_serial
    sim2real_msg
    livelybot_logger
    #hightorque_hardware_sdk
)

#find_package(hightorque_hardware_sdk REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)

message("PINOCCHIO include directories: ${PINOCCHIO_INCLUDE_DIRS}")
message("PINOCCHIO libraries: ${pinocchio_LIBRARIES}")
message("OpenVINO Include Dirs: ${OpenVINO_INCLUDE_DIRS}")
message("OpenVINO Library Dirs: ${OpenVINO_LIBRARIES}")


find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

catkin_package(
    LIBRARIES ${PROJECT_NAME}_lib  
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}  ${PINOCCHIO_INCLUDE_DIRS}
    DEPENDS rt pthread pinocchio 
    CATKIN_DEPENDS roscpp std_msgs livelybot_msg serial livelybot_serial sim2real_msg livelybot_logger #hightorque_hardware_sdk
)

message("livelybot_serial_INCLUDE_DIRS : ${livelybot_serial_INCLUDE_DIRS}")
message("hightorque_hardware_sdk_INCLUDE_DIRS  : ${hightorque_hardware_sdk_INCLUDE_DIRS}")
include_directories(
  ${PINOCCHIO_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenVINO_INCLUDE_DIRS}
  ${RKNN_PATH}
  ${livelybot_serial_INCLUDE_DIRS}
  /opt/ros/noetic/include
  ../../devel/include #hightorque_hardware_sdk_include
  ../sim2real_master/include
  ../livelybot_logger/include
  ../../install/include/livelybot_serial/  #for install
  ../../install/include/livelybot_serial/crc  #for install
  ../../install/include/livelybot_serial/hardware #for install
)

add_compile_options(-Wno-ignored-attributes)


set(RL_CTRL_SRCS
  src/rl_controller.cpp 
  src/read_rl_yaml.cpp 
  include/read_rl_yaml.h
  include/rl_controller.h
)

set(PD_CTRL_SRCS
  include/jacobian.h
  include/robot_data.h
  include/pd_controller.h
  include/read_pd_yaml.h
  include/kinematics.h
  src/jacobian.cpp
  src/pd_controller.cpp 
  src/read_pd_yaml.cpp 
  src/kinematics.cpp 
  src/mapping.cpp
  ../sim2real_master/src/CubicSplineInterpolator.cpp
)

link_directories(${PROJECT_SOURCE_DIR}/lib)
add_executable(rl_pd_controller  src/main.cpp ${RL_CTRL_SRCS} ${PD_CTRL_SRCS})

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(rl_pd_controller
  #libsim2real_arm_lib.so 
  #libsim2real_master_arm_lib.so 
  ${catkin_LIBRARIES} 
  ${SERIALPORT_LIBRARY} 
  yaml-cpp 
  ${pinocchio_LIBRARIES}
  ${RKNN_PATH}/librknn_api.so
  ${RKNN_PATH}/librknnrt.so
  )
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_libraries(rl_pd_controller
  #libsim2real_x86_lib.so #避免使用自己生成的so文件 
  ${catkin_LIBRARIES} 
  ${SERIALPORT_LIBRARY} 
  yaml-cpp 
  ${pinocchio_LIBRARIES}
  ${OpenVINO_LIBRARIES}
  /usr/lib/libopenvino.so
  )
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
add_library(${PROJECT_NAME}_arm_lib ${RL_CTRL_SRCS} ${PD_CTRL_SRCS}) 
target_link_libraries(${PROJECT_NAME}_arm_lib PUBLIC ${catkin_LIBRARIES} 
${SERIALPORT_LIBRARY} yaml-cpp ${pinocchio_LIBRARIES} ${livelybot_serial} )
target_include_directories( ${PROJECT_NAME}_arm_lib PUBLIC  include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR}
${PINOCCHIO_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
add_library(${PROJECT_NAME}_x86_lib ${RL_CTRL_SRCS} ${PD_CTRL_SRCS}) 
target_link_libraries(${PROJECT_NAME}_x86_lib PUBLIC ${catkin_LIBRARIES} 
${SERIALPORT_LIBRARY} yaml-cpp ${pinocchio_LIBRARIES} ${livelybot_serial} ${OpenVINO_LIBRARIES})
target_include_directories( ${PROJECT_NAME}_x86_lib PUBLIC  include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR}
${PINOCCHIO_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${OpenVINO_INCLUDE_DIRS}
)
endif()


install(TARGETS rl_pd_controller
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY policy/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/policy
)


install(DIRECTORY way_point/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/way_point
)

install(DIRECTORY resources/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
#install(TARGETS ${PROJECT_NAME}_x86_lib
install(TARGETS ${PROJECT_NAME}_arm_lib
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
install(TARGETS ${PROJECT_NAME}_x86_lib
#install(TARGETS ${PROJECT_NAME}_arm_lib
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
endif()

# get_target_property(LINK_LIBS ${PROJECT_NAME}_lib LINK_LIBRARIES)
# message("Link libraries for ${PROJECT_NAME}_lib: ")
# foreach(a ${LINK_LIBS})
# message(${a})
# endforeach(a ${${LINK_LIBS}})

cmake_minimum_required(VERSION 3.10)
project(external_multirotor_ardupilot_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp roslib geometry_msgs)

set(UPSTREAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/upstream")
set(PROTOBUF_IMPORT_DIRS /usr/include/gazebo-11/gazebo/msgs/proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  ${UPSTREAM_DIR}/msgs/CommandMotorSpeed.proto
  ${UPSTREAM_DIR}/msgs/MotorSpeed.proto
  ${UPSTREAM_DIR}/msgs/Float.proto
  ${UPSTREAM_DIR}/msgs/Wind.proto)

add_library(px4_motor_msgs SHARED ${PROTO_SRCS})
target_include_directories(px4_motor_msgs PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  /usr/include/gazebo-11/gazebo/msgs)
target_link_libraries(px4_motor_msgs PUBLIC ${Protobuf_LIBRARIES} gazebo_msgs)

add_library(gazebo_motor_model SHARED
  ${UPSTREAM_DIR}/gazebo_motor_model.cpp)
target_include_directories(gazebo_motor_model PRIVATE
  ${UPSTREAM_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  /usr/include/eigen3)
target_link_directories(gazebo_motor_model PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(gazebo_motor_model PRIVATE
  px4_motor_msgs ${GAZEBO_LIBRARIES} tinyxml)

add_library(ArduPilotMotorBridge SHARED ArduPilotMotorBridge.cc)
target_include_directories(ArduPilotMotorBridge PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS})
target_link_directories(ArduPilotMotorBridge PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(ArduPilotMotorBridge PRIVATE
  px4_motor_msgs ${GAZEBO_LIBRARIES})

# CTU-MRS RoboFly modelinin resmi govde surukleme eklentisi. Kaynak dosya
# upstream commitinden degistirilmeden vendored edilir (bkz. THIRD_PARTY.md).
add_library(MrsGazeboCommonResources_FluidResistancePlugin SHARED
  ${UPSTREAM_DIR}/fluid_resistance_plugin.cpp)
target_include_directories(MrsGazeboCommonResources_FluidResistancePlugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})
target_link_directories(MrsGazeboCommonResources_FluidResistancePlugin PRIVATE
  ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(MrsGazeboCommonResources_FluidResistancePlugin PRIVATE
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES})

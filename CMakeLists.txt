cmake_minimum_required(VERSION 2.8.3)
project(kinova_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller_manager
  hardware_interface
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kinova_HI
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(low_level_lib STATIC
	src/jaco2_rs485.cpp
	)

add_executable(kinovaHI src/kinovaHI.cpp)
target_link_libraries(kinovaHI ${catkin_LIBRARIES})

add_executable(kinovaFakeHI src/kinovaFakeHI.cpp)
target_link_libraries(kinovaFakeHI ${catkin_LIBRARIES})

add_executable(kinovaLowLevelHI src/kinovaLowLevelHI.cpp)
target_link_libraries(kinovaLowLevelHI low_level_lib ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(kinovaVelHI src/kinovaHI_vel.cpp)
target_link_libraries(kinovaVelHI ${catkin_LIBRARIES})

add_executable(talker_frc src/talker_frc.cpp)
target_link_libraries(talker_frc ${catkin_LIBRARIES})

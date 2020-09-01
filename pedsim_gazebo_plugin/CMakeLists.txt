cmake_minimum_required(VERSION 2.8.3)
project(pedsim_gazebo_plugin)
add_definitions(-Wall -Wunused -std=c++11 -pipe -Wunused -Wextra)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(PACKAGE_DEPS
    gazebo_ros
    geometry_msgs
    roscpp
    rospy
    gazebo_msgs
    pedsim_msgs
    pedsim_utils)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPS})

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
# if(gazebo_VERSION_MAJOR GREATER 8) ign_import_target(UUID) message(STATUS
# "Applying patch to ${PROJECT_NAME} to avoid error with GZ9: Target UUID::UUID
# not found.") endif()
link_directories(${GAZEBO_LIBRARY_DIRS})

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# ##############################################################################
# Declare ROS dynamic reconfigure parameters ##
# ##############################################################################

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################

catkin_package(
  # INCLUDE_DIRS include LIBRARIES gzb_vel_plugin
  CATKIN_DEPENDS ${PACKAGE_DEPS} # DEPENDS system_lib
)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(ActorPosesPlugin src/actor_poses_plugin.cpp)
target_link_libraries(ActorPosesPlugin ${catkin_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################

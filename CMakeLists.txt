cmake_minimum_required(VERSION 2.8.3)
project(swarm_uav_manipulator)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -I /usr/include/eigen3")
endif()

find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED rosjava_build_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  hector_uav_msgs
  control_toolbox
)

find_package(ompl)


# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp publish)

catkin_package(CATKIN_DEPENDS roscpp rospy tf std_msgs hector_uav_msgs)



include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OMPL_LIBRARIES})

add_executable(motion_planner_ompl3D src/motion_planner_ompl3D.cpp src/PlannerCallbacks.cpp)
target_link_libraries(motion_planner_ompl3D ${catkin_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(quad_manipulator  src/quad_manipulator.cpp src/ControllerCallbacks.cpp)
target_link_libraries(quad_manipulator ${catkin_LIBRARIES})

add_executable(collision_checker src/collision_checker.cpp)
target_link_libraries(collision_checker ${catkin_LIBRARIES})

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})

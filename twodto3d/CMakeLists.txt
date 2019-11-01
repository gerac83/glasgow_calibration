cmake_minimum_required(VERSION 2.8.3)
project(twodto3d)
find_package(PCL 1.2 REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  tf
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  
 
)

add_message_files(
  FILES
  mymessage.msg
)


add_service_files(
  FILES
  twodto3d.srv

)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  trajectory_msgs
  
  

)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(INCLUDE_DIRS 
  CATKIN_DEPENDS
  
LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}

)

link_directories(
	${catkin_LIBRARY_DIRS}

)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS}
${qt4_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(twodto3d src/twodto3d.cpp)
add_dependencies(twodto3d ${PROJECT_NAME}_gencpp)
target_link_libraries(twodto3d ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})



add_executable(transform_pub src/transform_pub.cpp)
add_dependencies(transform_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_pub ${catkin_LIBRARIES})













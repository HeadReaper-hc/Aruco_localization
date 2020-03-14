cmake_minimum_required(VERSION 2.8.3)
project(aruco_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
)

## find opencv
find_package(OpenCV REQUIRED)
## find sophus
find_package(Sophus REQUIRED)
## find g2o
find_package(G2O REQUIRED)
## find CSparse
find_package(CSparse REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco_localization
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  "/usr/include/eigen3"
  "include/aruco_localization"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
)

file(GLOB SRC_FILE src/*.cpp)

set(THIRD_LIBS g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${CSPARSE_LIBRARY}
        )
add_executable(aruco_node test/aruco_node.cpp ${SRC_FILE})
target_link_libraries(aruco_node ${catkin_LIBRARIES} ${THIRD_LIBS})


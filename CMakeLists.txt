cmake_minimum_required(VERSION 3.0.2)
project(husky_lqr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include/${PROJECT_NAME}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
  $ENV{EIGEN3_INCLUDE_DIRS}
  # ${manif_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/Lqr.cpp
)

## Declare a C++ executable
add_executable(lqr_controller
  src/controller.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Link the executable to the library just created.
target_link_libraries(lqr_controller
  ${PROJECT_NAME}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
)
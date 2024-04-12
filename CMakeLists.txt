cmake_minimum_required(VERSION 3.5)
project(reloPush LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
)

# Find Boost
find_package(Boost REQUIRED)
if(NOT Boost_FOUND)
    message(AUTHOR_WARNING,"Boost not found")
endif(NOT Boost_FOUND)

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "\n Eigen 3 NOT FOUND \n")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reloPush
#  CATKIN_DEPENDS eigen_conversions geometry_msgs message_generation nav_msgs roscpp rospy std_msgs tf tf_conversions
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#test programs
add_executable(test_dubins "test/test_dubins.cpp")
target_link_libraries(test_dubins ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(${PROJECT_NAME} "src/reloPush.cpp" "src/graph_info.cpp")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES})



install(TARGETS test_dubins
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
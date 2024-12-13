cmake_minimum_required(VERSION 3.5)
project(reloPush LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CMake Path
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/config.h @ONLY)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} src/pathPlanTools)

#test programs
add_executable(test_dubins "test/test_dubins.cpp")
target_link_libraries(test_dubins ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(test_hastar "test/test_hastar.cpp" "src/reloPush/params.cpp" "src/pathPlanTools/path_planning_tools.cpp" "src/pathPlanTools/tf_tools.inl"
    "src/pathPlanTools/tf_tools.cpp")
target_link_libraries(test_hastar ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(mocap_tf "src/mocap_tf.cpp" 
                "src/reloPush/params.cpp" 
                "src/pathPlanTools/path_planning_tools.cpp" 
                "src/pathPlanTools/tf_tools.inl" 
                "src/pathPlanTools/tf_tools.cpp"
                "src/reloPush/hexNfloat.cpp")
target_link_libraries(mocap_tf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(
    ${PROJECT_NAME} 
    "src/reloPush.cpp" 
    "src/graph_info.cpp"  
    "src/edge_construction.cpp" 
    "src/reloPush/movableObject.cpp"
    "src/omplTools/fromOMPL.cpp"
    "src/omplTools/dubins_tools.cpp"
    "src/dijkstra_tools.cpp"
    "src/reloPush/color_print.cpp"
    "src/reloPush/parse_testdata.cpp"
    "src/pathPlanTools/tf_tools.inl"
    "src/pathPlanTools/tf_tools.cpp"
    "src/pathPlanTools/path_planning_tools.cpp"
    "src/reloPush/params.cpp"
    "src/reloPush/push_pose_tools.cpp"
    "src/reloPush/data_collector.cpp"
    "src/reloPush/hexNfloat.cpp"
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES})

install(TARGETS test_dubins
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS test_hastar
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
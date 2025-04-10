cmake_minimum_required(VERSION 3.5)

project(ReloPush-Rebuild LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Add ZeroMQ cmake file
list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/config.h @ONLY)


find_package(Boost REQUIRED)

if(NOT Boost_FOUND)
    message(FATAL_ERROR "Boost not found")
endif()

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
    message(FATAL_ERROR "\n Eigen 3 NOT FOUND \n")
endif()

find_package(GTest 1.14.0 REQUIRED)

find_package(Ceres REQUIRED)

## Path Visualization
find_package(Qt5 COMPONENTS Widgets REQUIRED)

#find_package(absl CONFIG REQUIRED)
# ZeroMQ
if (NOT TARGET libzmq AND NOT TARGET libzmq-static)
  find_package(ZeroMQ QUIET)

  # libzmq autotools install: fallback to pkg-config
  if(NOT ZeroMQ_FOUND)
    message(STATUS "CMake libzmq package not found, trying again with pkg-config (normal install of zeromq)")
    list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/libzmq-pkg-config)
    find_package(ZeroMQ REQUIRED)
  endif()

  # TODO "REQUIRED" above should already cause a fatal failure if not found, but this doesn't seem to work
  if(NOT ZeroMQ_FOUND)
    message(FATAL_ERROR "ZeroMQ was not found, neither as a CMake package nor via pkg-config")
  endif()

  #if (ZeroMQ_FOUND AND NOT (TARGET libzmq OR TARGET libzmq-static))
  #  message(FATAL_ERROR "ZeroMQ version not supported!")
  #endif()
endif()

include_directories(include ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${ZeroMQ_INCLUDE_DIRS})

## ODE ##
if(APPLE)
    # Set the path to the ODE library (MacOS specific)
    set(ODE_DIR "/opt/homebrew/opt/ode")
    include_directories(${ODE_DIR}/include)
    link_directories(${ODE_DIR}/lib)
endif()

add_executable(ReloPush src/main.cpp
    src/GraphBuilder.cpp
    src/InputParser.cpp
    src/DubinsTools.cpp
    src/FromOMPL.cpp
    src/TfTools.cpp
    src/params.cpp
    src/PathPlanningTools.cpp
    include/TaskAllocation.hpp
    src/TaskAllocartion.cpp
    src/PlanningContext.cpp
    src/PreReloOptimization.cpp
    src/PlanHybridAstar.cpp
    src/Visualization/VisualizationWidget.cpp
    src/Visualization/QtPathWindow.cpp
    src/Visualization/QtPathManager.cpp
    src/Visualization/QtMainControlWindow.cpp
    src/PrintInColor.cpp
    src/BatchInstanceParcer.cpp
)
if(APPLE)
    target_link_libraries(ReloPush ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} Qt5::Widgets Ceres::ceres ${ODE_DIR}/lib/libode.dylib)
else()
    target_link_libraries(ReloPush ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} Qt5::Widgets ${CERES_LIBRARIES} ${ZeroMQ_LIBRARIES} absl::log_initialize)
endif()

# Dubins Path Planning
add_executable(test_dubins test/test_dubins.cpp)
if(APPLE)
    target_link_libraries(test_dubins ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${ODE_DIR}/lib/libode.dylib)
else()
    target_link_libraries(test_dubins ${OMPL_LIBRARIES} ${Boost_LIBRARIES} absl::log_initialize)
endif()

# Hybrid Astar Planning
add_executable(test_hybrid_astar test/test_hybrid_astar.cpp
    src/PathPlanningTools.cpp
    src/FromOMPL.cpp
    src/Visualization/VisualizationWidget.cpp
    src/PlanHybridAstar.cpp
    src/PlanningContext.cpp)
if(APPLE)
    target_link_libraries(test_hybrid_astar ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${ODE_DIR}/lib/libode.dylib Qt5::Widgets)
else()
    target_link_libraries(test_hybrid_astar ${Boost_LIBRARIES} ${OMPL_LIBRARIES} Qt5::Widgets absl::log_initialize)
endif()

# Pre-relocation Optimization
add_executable(test_prerelo_opt test/test_prerelo_optimization.cpp src/PreReloOptimization.cpp src/FromOMPL.cpp)
if(APPLE)
    target_link_libraries(test_prerelo_opt Ceres::ceres ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} ${ODE_DIR}/lib/libode.dylib)
else()
     target_link_libraries(test_prerelo_opt ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} absl::log_initialize)
endif()

add_executable(test_parse_instance
    test/test_parse_instance.cpp
    src/GraphBuilder.cpp
    src/InputParser.cpp
    src/DubinsTools.cpp
    src/FromOMPL.cpp
    src/TfTools.cpp
    src/params.cpp
    src/PathPlanningTools.cpp
    include/TaskAllocation.hpp
    src/TaskAllocartion.cpp
    src/PlanningContext.cpp
    src/PreReloOptimization.cpp
    src/PlanHybridAstar.cpp
    src/Visualization/VisualizationWidget.cpp
    src/Visualization/QtPathWindow.cpp
    src/Visualization/QtPathManager.cpp
    src/Visualization/QtMainControlWindow.cpp
    src/PrintInColor.cpp
    src/BatchInstanceParcer.cpp)

if(APPLE)
    target_link_libraries(test_parse_instance ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${ODE_DIR}/lib/libode.dylib Qt5::Widgets Ceres::ceres)
else()
    target_link_libraries(test_parse_instance ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} Qt5::Widgets ${CERES_LIBRARIES} absl::log_initialize)
endif()

add_executable(test_trajectory_gen test/test_trajectory_gen.cpp)
target_link_libraries(test_trajectory_gen ${Boost_LIBRARIES} ${Eigen_LIBRARIES} Qt5::Widgets)

add_executable(test_trajectory_vis test/test_trajectory_gen.cpp)
target_link_libraries(test_trajectory_vis ${Boost_LIBRARIES} ${Eigen_LIBRARIES} Qt5::Widgets)


# test ZeroMQ
add_executable(test_zeromq test/test_zeromq.cpp src/BinaryString.cpp src/base64.cpp)
target_link_libraries(test_zeromq ${ZeroMQ_LIBRARIES})


include(GNUInstallDirs)
install(TARGETS ReloPush
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS test_dubins
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS test_hybrid_astar
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS test_prerelo_opt
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS test_parse_instance
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS test_trajectory_gen
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS test_zeromq
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

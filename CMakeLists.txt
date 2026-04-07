cmake_minimum_required(VERSION 3.5)

project(ReloPush-Rebuild LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Add ZeroMQ cmake file
if(UNIX)
    list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
elseif(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
endif()

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/include/ReloPush/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ReloPush/config.h @ONLY)


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
# First, try to find Qt6 with the Widgets component quietly.
find_package(Qt6 COMPONENTS Widgets QUIET)

if (Qt6_FOUND)
    message(STATUS "Found Qt6, using Qt6::Widgets")
    set(QT_WIDGET_TARGET Qt6::Widgets)
    # Optionally, define a preprocessor symbol to help your source code distinguish versions.
    add_compile_definitions(USE_QT6)
else()
    message(STATUS "Qt6 not found; trying Qt5")
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    message(STATUS "Found Qt5, using Qt5::Widgets")
    set(QT_WIDGET_TARGET Qt5::Widgets)
    add_compile_definitions(USE_QT5)
endif()

if(UNIX)
    find_package(absl CONFIG REQUIRED)
endif()


# ZeroMQ
if(APPLE)
    find_path(ZMQ_INCLUDE_DIR zmq.h PATHS /opt/homebrew/include)
    find_library(ZMQ_LIB NAMES zmq PATHS /opt/homebrew/lib)

    if(NOT ZMQ_INCLUDE_DIR OR NOT ZMQ_LIB)
      message(FATAL_ERROR "ZeroMQ was not found. Install with 'brew install zeromq'.")
    endif()

    include_directories(${ZMQ_INCLUDE_DIR})
else()
    if (NOT TARGET libzmq AND NOT TARGET libzmq-static)
        message(STATUS "Trying to find ZeroMQ")
      find_package(ZeroMQ QUIET)

      # libzmq autotools install: fallback to pkg-config
      if(NOT ZeroMQ_FOUND)
        message(STATUS "CMake libzmq package not found, trying again with pkg-config (normal install of zeromq)")
        list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/libzmq-pkg-config)
        if(APPLE)
            find_package(ZeroMQ REQUIRED)
        else()
            find_package(ZeroMQ REQUIRED)
        endif()
      endif()

      # TODO "REQUIRED" above should already cause a fatal failure if not found, but this doesn't seem to work
      if(NOT ZeroMQ_FOUND)
        message(FATAL_ERROR "ZeroMQ was not found, neither as a CMake package nor via pkg-config")
      endif()

      #if (ZeroMQ_FOUND AND NOT (TARGET libzmq OR TARGET libzmq-static))
      #  message(FATAL_ERROR "ZeroMQ version not supported!")
      #endif()
    endif()
endif()

include_directories(include include/ReloPush ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${ZeroMQ_INCLUDE_DIRS})
message(STATUS ${ZeroMQ_INCLUDE_DIRS})

## ODE ##
if(APPLE)
    # Set the path to the ODE library (MacOS specific)
    set(ODE_DIR "/opt/homebrew/opt/ode")
    include_directories(${ODE_DIR}/include)
    link_directories(${ODE_DIR}/lib)
endif()

add_executable(ReloPush-BOSS src/main.cpp
    src/ReloPush/GraphBuilder.cpp
    src/ReloPush/InputParser.cpp
    src/ReloPush/DubinsTools.cpp
    src/ReloPush/FromOMPL.cpp
    src/ReloPush/TfTools.cpp
    src/ReloPush/params.cpp
    src/ReloPush/PathPlanningTools.cpp
    include/ReloPush/TaskAllocation.hpp
    src/ReloPush/TaskAllocartion.cpp
    src/ReloPush/PlanningContext.cpp
    src/ReloPush/PreReloOptimization.cpp
    src/ReloPush/PlanHybridAstar.cpp
    src/ReloPush/Visualization/VisualizationWidget.cpp
    src/ReloPush/Visualization/QtPathWindow.cpp
    src/ReloPush/Visualization/QtPathManager.cpp
    src/ReloPush/Visualization/QtMainControlWindow.cpp
    src/ReloPush/PrintInColor.cpp
    src/ReloPush/BatchInstanceParcer.cpp
    src/ReloPush/Visualization/TrajectoryView.cpp
    include/ReloPush/Visualization/TrajectoryView.h
    src/ReloPush/BinaryString.cpp
    src/ReloPush/base64.cpp
)
if(APPLE)
    target_link_libraries(ReloPush-BOSS ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} ${QT_WIDGET_TARGET} ${ZMQ_LIB} Ceres::ceres ${ODE_DIR}/lib/libode.dylib)
else()
    target_link_libraries(ReloPush-BOSS ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} ${QT_WIDGET_TARGET} ${CERES_LIBRARIES} ${ZeroMQ_LIBRARIES} absl::log_initialize)
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
    src/ReloPush/PathPlanningTools.cpp
    src/ReloPush/FromOMPL.cpp
    src/ReloPush/Visualization/VisualizationWidget.cpp
    src/ReloPush/PlanHybridAstar.cpp
    src/ReloPush/PlanningContext.cpp)
if(APPLE)
    target_link_libraries(test_hybrid_astar ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${ODE_DIR}/lib/libode.dylib ${QT_WIDGET_TARGET})
else()
    target_link_libraries(test_hybrid_astar ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${QT_WIDGET_TARGET} absl::log_initialize)
endif()

# Pre-relocation Optimization
add_executable(test_prerelo_opt test/test_prerelo_optimization.cpp src/ReloPush/PreReloOptimization.cpp src/ReloPush/FromOMPL.cpp)
if(APPLE)
    target_link_libraries(test_prerelo_opt Ceres::ceres ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} ${ODE_DIR}/lib/libode.dylib)
else()
     target_link_libraries(test_prerelo_opt ${CERES_LIBRARIES} ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} absl::log_initialize)
endif()

add_executable(test_parse_instance
    test/test_parse_instance.cpp
    src/ReloPush/GraphBuilder.cpp
    src/ReloPush/InputParser.cpp
    src/ReloPush/DubinsTools.cpp
    src/ReloPush/FromOMPL.cpp
    src/ReloPush/TfTools.cpp
    src/ReloPush/params.cpp
    src/ReloPush/PathPlanningTools.cpp
    include/ReloPush/TaskAllocation.hpp
    src/ReloPush/TaskAllocartion.cpp
    src/ReloPush/PlanningContext.cpp
    src/ReloPush/PreReloOptimization.cpp
    src/ReloPush/PlanHybridAstar.cpp
    src/ReloPush/Visualization/VisualizationWidget.cpp
    src/ReloPush/Visualization/QtPathWindow.cpp
    src/ReloPush/Visualization/QtPathManager.cpp
    src/ReloPush/Visualization/QtMainControlWindow.cpp
    src/ReloPush/PrintInColor.cpp
    src/ReloPush/BatchInstanceParcer.cpp)

if(APPLE)
    target_link_libraries(test_parse_instance ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${ODE_DIR}/lib/libode.dylib ${QT_WIDGET_TARGET} Ceres::ceres)
else()
    target_link_libraries(test_parse_instance ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${QT_WIDGET_TARGET} ${CERES_LIBRARIES} absl::log_initialize)
endif()

add_executable(test_trajectory_gen test/test_trajectory_gen.cpp)
target_link_libraries(test_trajectory_gen ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${QT_WIDGET_TARGET})

add_executable(test_trajectory_vis test/test_trajectory_gen.cpp)
target_link_libraries(test_trajectory_vis ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${QT_WIDGET_TARGET})


# test ZeroMQ
add_executable(test_zeromq test/test_zeromq.cpp src/ReloPush/BinaryString.cpp src/ReloPush/base64.cpp)
if(APPLE)
    target_link_libraries(test_zeromq ${ZMQ_LIB})
else()
    target_link_libraries(test_zeromq ${ZeroMQ_LIBRARIES})
endif()

# test loading finalSequence from file
add_executable(test_load_finalSequence test/test_load_finalSequence.cpp src/ReloPush/BinaryString.cpp src/ReloPush/base64.cpp src/ReloPush/PathPlanningTools.cpp)
target_link_libraries(test_load_finalSequence ${Boost_LIBRARIES} ${OMPL_LIBRARIES})



include(GNUInstallDirs)
install(TARGETS ReloPush-BOSS
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
install(TARGETS test_load_finalSequence
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

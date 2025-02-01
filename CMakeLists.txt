cmake_minimum_required(VERSION 3.5)

project(ReloPush-Rebuild LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


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

find_package(Ceres REQUIRED)

## Path Visualization
find_package(Qt6 COMPONENTS Widgets REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

## ODE ##
if(APPLE)
    # Set the path to the ODE library (MacOS specific)
    set(ODE_DIR "/opt/homebrew/opt/ode")
    include_directories(${ODE_DIR}/include)
    link_directories(${ODE_DIR}/lib)
endif()

add_executable(ReloPush src/main.cpp
    src/GraphBuilder.cpp
    src/InputParser.hpp
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
)
target_link_libraries(ReloPush ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} Qt6::Widgets Ceres::ceres ${ODE_DIR}/lib/libode.dylib)

# Dubins Path Planning
add_executable(test_dubins test/test_dubins.cpp)
target_link_libraries(test_dubins ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${ODE_DIR}/lib/libode.dylib)

# Hybrid Astar Planning
add_executable(test_hybrid_astar test/test_hybrid_astar.cpp
    src/PathPlanningTools.cpp
    src/FromOMPL.cpp
    src/Visualization/VisualizationWidget.cpp
    src/PlanHybridAstar.cpp
    src/PlanningContext.cpp)
target_link_libraries(test_hybrid_astar ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${ODE_DIR}/lib/libode.dylib Qt6::Widgets)

# Pre-relocation Optimization
add_executable(test_prerelo_opt test/test_prerelo_optimization.cpp src/PreReloOptimization.cpp)
target_link_libraries(test_prerelo_opt Ceres::ceres ${Boost_LIBRARIES} ${OMPL_LIBRARIES} ${Eigen_LIBRARIES} ${ODE_DIR}/lib/libode.dylib)

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

cmake_minimum_required(VERSION 3.5)
project(mpc_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MPC_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/ReloPush
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

set(MPC_SOURCES
    src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BinaryString.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
)

add_executable(mpc_controller ${MPC_SOURCES})

target_include_directories(mpc_controller PRIVATE ${MPC_INCLUDE_DIRS})

# Link libraries based on platform (matching root CMakeLists.txt)
if(APPLE)
    target_link_libraries(mpc_controller
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
            Ceres::ceres
    )
else()
    target_link_libraries(mpc_controller
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
            ${CERES_LIBRARIES}
    )
endif()

# Dummy straight mover executable for real-robot VESC testing
set(DUMMY_MOVER_SOURCES
    src/dummy_straight_mover.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
)
add_executable(mpc_dummy_straight_mover ${DUMMY_MOVER_SOURCES})
target_include_directories(mpc_dummy_straight_mover PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(mpc_dummy_straight_mover
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(mpc_dummy_straight_mover
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
    )
endif()

install(TARGETS mpc_controller mpc_dummy_straight_mover
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

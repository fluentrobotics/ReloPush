cmake_minimum_required(VERSION 3.5)
project(mpc_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MPC_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/ReloPush
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

# Shared kinematic-bicycle plant model (mpc::State4/rollout_step/
# predict_delay_compensated). No Ceres dependency -- used by BOTH MpcCore
# (the real MPC solver) and SimCore (the standalone simulator) so the two
# never duplicate the equations.
set(MPC_KINEMATICS_SOURCES
    src/Kinematics.cpp
)

set(MPC_CORE_SOURCES
    src/RobotSpec.cpp
    ${MPC_KINEMATICS_SOURCES}
    src/MpcCore.cpp
)

# Pure sim logic (ackermann decode, localization encode, clamping, watchdog
# state machine) for mpc_robot_sim. Depends only on RobotSpec + Kinematics
# (no Ceres), so mpc_robot_sim never needs to link Ceres.
set(SIM_CORE_SOURCES
    src/RobotSpec.cpp
    ${MPC_KINEMATICS_SOURCES}
    src/SimCore.cpp
)

set(MPC_SOURCES
    src/main.cpp
    ${MPC_CORE_SOURCES}
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

# Standalone kinematic (no-dynamics) Ackermann robot simulator. Presents the
# same two ZMQ endpoints mpc_controller already talks to (see
# src/robot_sim.cpp's header comment) so sim<->hardware is an endpoint-
# configuration swap. Deliberately does NOT link Ceres -- SIM_CORE_SOURCES
# only pulls in RobotSpec + the shared Kinematics model.
set(SIM_SOURCES
    src/robot_sim.cpp
    ${SIM_CORE_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
)
add_executable(mpc_robot_sim ${SIM_SOURCES})
target_include_directories(mpc_robot_sim PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(mpc_robot_sim
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(mpc_robot_sim
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
    )
endif()

# Unit tests for the mpc:: core math (RobotSpec/MpcCore/SimCore), no ZMQ
# sockets involved -- mirrors the lightweight harness style of
# MARS/tests/phastar_unit_tests.cpp (no gtest dependency).
set(MPC_UNIT_TEST_SOURCES
    tests/mpc_unit_tests.cpp
    ${MPC_CORE_SOURCES}
    src/SimCore.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BinaryString.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
)
add_executable(mpc_unit_tests ${MPC_UNIT_TEST_SOURCES})
target_include_directories(mpc_unit_tests PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(mpc_unit_tests
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
            Ceres::ceres
    )
else()
    target_link_libraries(mpc_unit_tests
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
            ${CERES_LIBRARIES}
    )
endif()

# Stage 2: standalone loopback integration test. Spawns real mpc_controller
# and mpc_robot_sim child processes (by path, resolved at runtime via
# /proc/self/exe) and drives the actual MARS REQ/REP handshake against
# mpc_controller's real REP socket -- exercises the live wire protocol
# end-to-end, not a mock. No ZMQ/Ceres math lives here; it only needs
# ReloPush::trajectory + base64 to build/encode the handshake payload the
# same way MARS does. Test-only ports (45111/45161/45261), never the
# production defaults.
add_executable(test_mpc_sim_loop
    tests/test_mpc_sim_loop.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BinaryString.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
)
target_include_directories(test_mpc_sim_loop PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(test_mpc_sim_loop
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(test_mpc_sim_loop
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
    )
endif()

# test_mpc_sim_loop is a standalone diagnostic binary (spawns real child
# processes and paces real time), deliberately NOT added to
# mpc_controller/mpc_robot_sim/mpc_unit_tests' install() list below -- it is
# built and run manually, mirroring how MARS/tests/test_mpc_full_loop.cpp is
# not installed either.

# FEATURE A: standalone closed-loop noise test. Mirrors test_mpc_sim_loop
# exactly (same spawn/handshake/CSV-readback pattern, on its own test-only
# port block 45711/45741/45761) but runs Scenario A's straight trajectory
# twice -- once with --noise-sigma-pct=0 (must reproduce test_mpc_sim_loop's
# tolerances) and once with actuation noise enabled -- to prove closed-loop
# behavior is unaffected at sigma=0 and that the MPC still converges (within
# a relaxed tolerance) with noise on.
add_executable(test_mpc_sim_noise
    tests/test_mpc_sim_noise.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BinaryString.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
)
target_include_directories(test_mpc_sim_noise PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(test_mpc_sim_noise
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(test_mpc_sim_noise
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
    )
endif()

# test_mpc_sim_noise is likewise a standalone diagnostic binary, not added to
# the install() list below.

install(TARGETS mpc_controller mpc_dummy_straight_mover mpc_robot_sim mpc_unit_tests
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

cmake_minimum_required(VERSION 3.10)
project(vesc_driver CXX)

# ---------------------------------------------------------------------
# HARD PORTABILITY RULES for everything in VescDriver/: this whole folder
# is copied VERBATIM to a Jetson Nano (ARM64, Ubuntu 18.04: GCC 7.5,
# CMake 3.10.2, apt libzmq3-dev). Keep it compatible with that toolchain:
#   - C++14 only (no C++17 features, no std::filesystem, no
#     std::optional/std::string_view).
#   - Only CMake 3.10-era commands (no FetchContent, no
#     target_link_options, no cxx_std_* compile features -- use
#     CMAKE_CXX_STANDARD, as below).
#   - ZeroMQ via the raw C API (zmq.h) only -- find_library(zmq) +
#     find_path(zmq.h); never cppzmq/zmq.hpp anywhere in this folder.
#   - POSIX/termios/dirent for system access; no x86-specific intrinsics.
#   - No #includes of anything outside VescDriver/ (own vendored
#     third_party/ copies of nlohmann::json and base64 -- see below).
# This file also builds standalone (mkdir build && cd build && cmake ..
# && make) AND via add_subdirectory(VescDriver) from the repo root
# (which sets CMAKE_CXX_STANDARD 17 for itself) -- CMAKE_CXX_STANDARD is
# a plain (non-cache) directory-scoped variable, so overriding it here
# only affects targets defined in this file, without leaking back up to
# the parent project.
# ---------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra)
endif()

# ---------------------------------------------------------------------
# ZeroMQ: raw C API only (zmq.h) -- part 3 (ZmqChannels/vesc_driver_main)
# is what actually needs it now: the driver's ackermann SUB / control REP /
# telemetry PUB sockets, plus vesc_integration_tests' pty+ZMQ integration test.
# A HARD requirement from here on (unlike parts 1/2, where it was merely
# "will be needed later") -- install with 'apt install libzmq3-dev' if this
# fails.
# ---------------------------------------------------------------------
find_path(VESC_ZMQ_INCLUDE_DIR NAMES zmq.h)
find_library(VESC_ZMQ_LIB NAMES zmq)
if(VESC_ZMQ_INCLUDE_DIR AND VESC_ZMQ_LIB)
    message(STATUS "VescDriver: found libzmq (${VESC_ZMQ_LIB}), headers in ${VESC_ZMQ_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "VescDriver: libzmq (zmq.h / libzmq) not found on this machine. Required "
                         "from this part onward (ZmqChannels/vesc_driver_main and "
                         "vesc_integration_tests' pty+ZMQ integration test) -- install with "
                         "'apt install libzmq3-dev' and re-configure.")
endif()

set(VESC_DRIVER_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# ---------------------------------------------------------------------
# vesc_core: static library. Sources are added incrementally as this
# driver is built out across multiple tasks -- part 1 contributed the
# VESC UART packet codec (VescProtocol) and the ackermann wire-payload
# codec (AckermannCodec); part 2 added the POSIX termios serial transport
# (SerialPort) and VESC port auto-discovery (PortDiscovery), plus the
# vendored base64 codec everything above rests on; part 3 (this one) adds
# the core control-tick logic (DriverCore -- pure, no sockets/serial of its
# own) and the raw-libzmq ZMQ endpoint wrapper (ZmqChannels). TeleopCore
# (vesc_teleop's own pure state machine -- mode/magnitude/drive/deadman/
# abort-latch, no I/O) was added alongside vesc_mcconf's McconfPatcher.
# Steering calibration (shared by vesc_teleop/vesc_driver/future MPC-side
# tools -- servo center/limits, persisted outside the repo tree since this
# whole folder is rsync --delete'd on deploy) was added as SteeringCalib.
# Later parts append their own sources to this same list/target rather
# than creating a second library.
# ---------------------------------------------------------------------
set(VESC_CORE_SOURCES
    ${VESC_DRIVER_ROOT}/src/VescProtocol.cpp
    ${VESC_DRIVER_ROOT}/src/AckermannCodec.cpp
    ${VESC_DRIVER_ROOT}/src/SerialPort.cpp
    ${VESC_DRIVER_ROOT}/src/PortDiscovery.cpp
    ${VESC_DRIVER_ROOT}/src/DriverCore.cpp
    ${VESC_DRIVER_ROOT}/src/ZmqChannels.cpp
    ${VESC_DRIVER_ROOT}/src/McconfPatcher.cpp
    ${VESC_DRIVER_ROOT}/src/TeleopCore.cpp
    ${VESC_DRIVER_ROOT}/src/SteeringCalib.cpp
    ${VESC_DRIVER_ROOT}/src/SpeedGovernor.cpp
    ${VESC_DRIVER_ROOT}/src/VelocityMap.cpp
    ${VESC_DRIVER_ROOT}/src/SteeringAngleMap.cpp
    ${VESC_DRIVER_ROOT}/third_party/base64.cpp
)

add_library(vesc_core STATIC ${VESC_CORE_SOURCES})
target_include_directories(vesc_core PUBLIC
    ${VESC_DRIVER_ROOT}/src
    ${VESC_DRIVER_ROOT}/third_party
    ${VESC_ZMQ_INCLUDE_DIR}
)
target_link_libraries(vesc_core PUBLIC ${VESC_ZMQ_LIB})

# ---------------------------------------------------------------------
# vesc_unit_tests / vesc_integration_tests: standalone, manually-run test
# binaries -- NOT add_test()'d and NOT added to any install() list,
# mirroring the documented convention for MPC/CMakeLists.txt's test
# binaries (see that file's own comment block). Split from a former
# single vesc_driver_tests binary so the common edit/build/run loop only
# pays for the fast, pure in-process half: vesc_unit_tests never forks,
# execs, or touches fake_vesc, so it builds and runs in seconds.
# vesc_integration_tests holds every test that spawns a process (pty,
# fake_vesc, the real vesc_teleop/vesc_driver/vesc_mcconf binaries) --
# run it when a change touches process spawning / wire behavior / CLI
# startup, and before deploying to the robot. Both share their harness
# via tests/test_support.h (header-only, no separate lib target).
# ---------------------------------------------------------------------
add_executable(vesc_unit_tests tests/vesc_unit_tests.cpp)
target_link_libraries(vesc_unit_tests PRIVATE vesc_core)

add_executable(vesc_integration_tests tests/vesc_integration_tests.cpp)
target_link_libraries(vesc_integration_tests PRIVATE vesc_core)

# ---------------------------------------------------------------------
# fake_vesc: hardware-free VESC stand-in used by vesc_integration_tests'
# pty integration tests (and usable manually too) -- opens a pty master,
# prints "PTY <slave_path>" on its first stdout line, then speaks the
# real VESC UART protocol on that pty so a vesc::SerialPort opened on the
# slave path can talk to it exactly as it would to real hardware. Also a
# standalone, manually-run binary: not add_test()'d, not installed (same
# convention as the test binaries above).
# ---------------------------------------------------------------------
add_executable(fake_vesc tests/fake_vesc.cpp)
target_link_libraries(fake_vesc PRIVATE vesc_core)

# ---------------------------------------------------------------------
# vesc_driver: the real driver binary (part 3) -- the thin ZMQ+serial I/O
# shell around DriverCore/ZmqChannels/VescProtocol/SerialPort/
# PortDiscovery/AckermannCodec. Copy this whole VescDriver/ folder
# (including config/driver_config.json) to the Jetson and run this binary
# there -- see README.md.
# ---------------------------------------------------------------------
add_executable(vesc_driver src/vesc_driver_main.cpp)
target_link_libraries(vesc_driver PRIVATE vesc_core)

# ---------------------------------------------------------------------
# vesc_mcconf: standalone motor-config tuning CLI (dump/scan/patch/
# restore speed-PID gains and current limits over USB, without VESC
# Tool). Talks to the VESC directly via SerialPort/PortDiscovery/
# VescProtocol -- must NOT be run at the same time as vesc_driver (the
# serial port is exclusive; the tool itself warns about this at startup).
# Same convention as the executables above: not add_test()'d, not
# installed.
# ---------------------------------------------------------------------
add_executable(vesc_mcconf src/vesc_mcconf_main.cpp)
target_link_libraries(vesc_mcconf PRIVATE vesc_core)

# ---------------------------------------------------------------------
# vesc_teleop: standalone interactive motor teleoperation CLI (first-
# contact testing of a new VESC against a bare, unloaded motor -- see
# README.md's "Teleoperation" section). Talks to the VESC directly via
# SerialPort/PortDiscovery/VescProtocol, same as vesc_mcconf/vesc_driver
# -- exclusive serial port, must not run alongside either of them. Links
# vesc_core only (no ZMQ needed for this tool, unlike vesc_driver). Same
# convention as the other executables: not add_test()'d, not installed.
# ---------------------------------------------------------------------
add_executable(vesc_teleop src/vesc_teleop_main.cpp)
target_link_libraries(vesc_teleop PRIVATE vesc_core)

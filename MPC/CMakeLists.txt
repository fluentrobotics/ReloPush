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
    src/LaunchGovernor.cpp
    src/DriverTelemetry.cpp
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

# FEATURE B Part A: standalone process-level deadband demo. Spawns a real
# mpc_robot_sim with --deadband directly -- NO mpc_controller in this
# scenario (mirrors test_mpc_sim_noise.cpp's run_clamp_check(): a throwaway
# PUB hand-drives the sim's real ackermann cmd socket) -- and reads back both
# the CSV log and the new telemetry topic (via the real
# mpc::parse_telemetry_payload(), SimCore.cpp linked directly below) to prove
# the deadband model end-to-end on the real binary. Test-only ports
# 45811/45841/45861. Package B (mpc_controller LAUNCH GOVERNOR) extended this
# binary with Parts B/C/D: an mpc_controller-in-the-loop scenario using the
# handshake port (45811, previously reserved/unused), which needs a real
# ReloPush::trajectory (serialize()/append_waypoint()) to drive -- hence
# BinaryString.cpp/BatchInstanceParcer.cpp joining base64.cpp below,
# mirroring test_mpc_sim_loop's own three-source ReloPush dependency list.
add_executable(test_mpc_deadband
    tests/test_mpc_deadband.cpp
    ${SIM_CORE_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BinaryString.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
)
target_include_directories(test_mpc_deadband PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(test_mpc_deadband
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(test_mpc_deadband
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
    )
endif()

# test_mpc_deadband is likewise a standalone diagnostic binary, not added to
# the install() list below.

# ---------------------------------------------------------------------
# OptiTrack/NatNet -> ZMQ localization bridge (v1, RECEIVE-TEST capability).
# Pure NatNet parse/build + up-axis geometry lives in OptiTrackCore.cpp (no
# sockets); optitrack_zmq_bridge.cpp/fake_motive.cpp are thin I/O shells
# around it, mirroring SimCore/robot_sim.cpp's split. These targets need zmq
# + nlohmann (vendored header-only, already on MPC_INCLUDE_DIRS) but NOT
# Ceres/Qt -- mirrors SIM_CORE_SOURCES' dependency-light footprint.
# mpc::encode_localization_payload is reused directly from SimCore.cpp
# (linked in below) so the bridge's wire bytes are byte-identical to
# mpc_robot_sim's, rather than re-deriving the JSON format locally.
set(OPTITRACK_CORE_SOURCES
    src/OptiTrackCore.cpp
    src/SimCore.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
)

# Mocap pose-jump filter (see doc/MOCAP_POSE_FILTER_PLAN.md): a per-body
# gated constant-velocity Kalman filter, pure logic (no ZMQ/JSON -- only
# Eigen, header-only via MPC_INCLUDE_DIRS' Eigen_INCLUDE_DIRS, no separate
# link target needed beyond the ${Eigen_LIBRARIES} every other Eigen-using
# target here already links). Kept as its own source list (not folded into
# OPTITRACK_CORE_SOURCES) since pose_filter_tests/pose_filter_replay need it
# WITHOUT the rest of OptiTrackCore's NatNet-parsing weight.
set(POSE_FILTER_SOURCES
    src/PoseFilter.cpp
)

add_executable(optitrack_zmq_bridge
    src/optitrack_zmq_bridge.cpp
    ${OPTITRACK_CORE_SOURCES}
    ${POSE_FILTER_SOURCES}
)
target_include_directories(optitrack_zmq_bridge PRIVATE ${MPC_INCLUDE_DIRS})
# Compiled-in default config directory: with no --mocap-config/--map-config
# flags, the bridge derives <dir>/mocap_config.txt and
# <dir>/mocap_map_config.json from this and loads them automatically if
# they exist (see mpc::resolve_config_path() in OptiTrackCore.h/.cpp for the
# pure resolution logic, and optitrack_zmq_bridge.cpp's main() for where
# this macro is consumed). CMAKE_CURRENT_SOURCE_DIR here is
# MPC/ (this file's own directory), so this resolves to
# MPC/config -- the real project config files' actual location.
target_compile_definitions(optitrack_zmq_bridge PRIVATE
    MOCAP_DEFAULT_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
)

add_executable(fake_motive
    src/fake_motive.cpp
    src/OptiTrackCore.cpp
)
target_include_directories(fake_motive PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(optitrack_zmq_bridge
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(optitrack_zmq_bridge
        PRIVATE
            ${Boost_LIBRARIES}
            ${Eigen_LIBRARIES}
            ${ZeroMQ_LIBRARIES}
    )
endif()
# fake_motive is UDP-only (no ZMQ) -- no ZeroMQ/ZMQ_LIB link needed.

# Unit tests for the mpc:: OptiTrackCore math (NatNet parse/build, up-axis
# geometry, Downsampler), no sockets involved -- mirrors mpc_unit_tests.cpp's
# lightweight harness style (no gtest dependency).
add_executable(optitrack_unit_tests
    tests/optitrack_unit_tests.cpp
    src/OptiTrackCore.cpp
)
target_include_directories(optitrack_unit_tests PRIVATE ${MPC_INCLUDE_DIRS})
# No ZMQ/Boost/Eigen/Ceres needed -- OptiTrackCore.cpp is pure C++ + nlohmann
# is header-only (see OPTITRACK_CORE_SOURCES comment above); this target is
# deliberately even lighter than that (doesn't even need SimCore/base64,
# since it never calls encode_localization_payload).

# Unit tests for mpc::parse_driver_telemetry (MPC/include/mpc/
# DriverTelemetry.h) -- the VESC driver telemetry JSON parser mpc_controller
# (main.cpp) uses to fill --log-csv's drv_*/telem_age_s columns. Pure
# logic, no sockets -- mirrors optitrack_unit_tests' dependency-light
# footprint (only nlohmann, header-only).
add_executable(driver_telemetry_tests
    tests/driver_telemetry_tests.cpp
    src/DriverTelemetry.cpp
)
target_include_directories(driver_telemetry_tests PRIVATE ${MPC_INCLUDE_DIRS})

# Process-level integration test: spawns real fake_motive + optitrack_zmq_bridge
# child processes over UDP loopback and SUBs to both robots' real localization
# PUB sockets, mirroring test_mpc_deadband.cpp's spawn/ProcessGuard structure.
# Drives already-built binaries via subprocess + ZMQ + JSON + file I/O only,
# so it does not need OptiTrackCore/SimCore/base64/Boost/Eigen linked in --
# only ZMQ + nlohmann. Test-only ports: UDP loopback 45510/45511, ZMQ
# 45861-45862 (see the file's own header comment). Standalone diagnostic
# binary, deliberately NOT added to the install() list below (mirrors
# test_mpc_deadband.cpp/test_mpc_sim_loop.cpp precedent).
add_executable(test_optitrack_bridge
    tests/test_optitrack_bridge.cpp
)
target_include_directories(test_optitrack_bridge PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(test_optitrack_bridge
        PRIVATE
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(test_optitrack_bridge
        PRIVATE
            ${ZeroMQ_LIBRARIES}
    )
endif()

# ---------------------------------------------------------------------
# pose_filter_tests: unit tests for mpc::PoseFilter (MPC/include/mpc/
# PoseFilter.h) -- deterministic synthetic 120Hz tracks, no sockets, no
# process spawning. Only needs Eigen (header-only) -- deliberately even
# lighter than optitrack_unit_tests (no nlohmann either).
# ---------------------------------------------------------------------
add_executable(pose_filter_tests
    tests/pose_filter_tests.cpp
    ${POSE_FILTER_SOURCES}
)
target_include_directories(pose_filter_tests PRIVATE ${MPC_INCLUDE_DIRS})
target_link_libraries(pose_filter_tests PRIVATE ${Eigen_LIBRARIES})

# ---------------------------------------------------------------------
# pose_filter_replay: offline tool that runs the SAME mpc::PoseFilter over a
# previously-recorded optitrack_zmq_bridge --log-csv file (per body), so the
# gate thresholds can be tuned/validated against real mocap data and a real
# captured jump can be turned into a regression fixture -- see doc/
# MOCAP_POSE_FILTER_PLAN.md's "Deliverables" #3/#4. Needs nlohmann
# (header-only, for the optional --map-config's "pose_filter" block) +
# Eigen, but no ZMQ/Boost/Ceres/Qt -- this is a pure CSV-in/stdout-out tool.
# ---------------------------------------------------------------------
add_executable(pose_filter_replay
    src/pose_filter_replay.cpp
    ${POSE_FILTER_SOURCES}
)
target_include_directories(pose_filter_replay PRIVATE ${MPC_INCLUDE_DIRS})
target_link_libraries(pose_filter_replay PRIVATE ${Eigen_LIBRARIES})

# ---------------------------------------------------------------------
# Motor calibration tool (main-PC side testable core). Pure logic (no ZMQ
# sockets -- see MotorCalibCore.h's file header comment for the
# dependency-injection design), needs only nlohmann (header-only, already on
# MPC_INCLUDE_DIRS) -- mirrors OPTITRACK_CORE_SOURCES/optitrack_unit_tests'
# dependency-light footprint (no Boost/Eigen/ZMQ/Ceres/Qt). No
# `motor_calibration` main() binary yet -- a later agent wires
# MotorCalibCore to a real ZMQ IDriverClient + OptiTrack pose feed; this
# only builds the unit-tested library piece.
add_executable(motor_calib_tests
    tests/motor_calib_tests.cpp
    src/MotorCalibCore.cpp
)
target_include_directories(motor_calib_tests PRIVATE ${MPC_INCLUDE_DIRS})
# motor_calib_tests is a standalone, manually-run test binary (see this
# project's test-binary convention below at test_mpc_sim_loop's own comment)
# -- no add_test(), not added to the install() list below.

# ---------------------------------------------------------------------
# motor_calibration: the real campaign-runner binary for the motor
# calibration tool -- wires MotorCalibCore's pure/tested TrialRunner to a
# real ZMQ IDriverClient (REQ to VescDriver's control REP) + a real
# OptiTrack pose feed (SUB) + the driver's own telemetry PUB. Needs
# cppzmq + nlohmann (header-only, already on MPC_INCLUDE_DIRS) but NOT
# Boost/Eigen/Ceres/Qt -- mirrors motor_calib_tests'/optitrack_unit_tests'
# dependency-light footprint.
# ---------------------------------------------------------------------
add_executable(motor_calibration
    src/motor_calibration.cpp
    src/CalibClient.cpp
    src/MotorCalibCore.cpp
)
target_include_directories(motor_calibration PRIVATE ${MPC_INCLUDE_DIRS})
# Compiled-in default config path: a zero-flag `motor_calibration` run loads
# this file automatically if it exists (see resolve_config() in
# motor_calibration.cpp) -- mirrors optitrack_zmq_bridge's
# MOCAP_DEFAULT_CONFIG_DIR compile definition, just pointing at a single
# file instead of a directory (this tool has exactly one config file).
target_compile_definitions(motor_calibration PRIVATE
    MOTOR_CALIB_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/motor_calib_config.json"
)

if(APPLE)
    target_link_libraries(motor_calibration
        PRIVATE
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(motor_calibration
        PRIVATE
            ${ZeroMQ_LIBRARIES}
    )
endif()

# ---------------------------------------------------------------------
# test_motor_calib_e2e: standalone, hardware-free end-to-end test. Spawns
# real fake_vesc (VescDriver/tests) + vesc_driver (VescDriver/src) child
# processes and plays OptiTrack itself (integrating the driver's own
# telemetry-reported erpm at a known ground-truth erpm_per_mps), then runs
# the real motor_calibration binary against them wrapped, asserting on its
# output files. Test-only ZMQ ports 45930-45949 ONLY (see the file's own
# header comment). Standalone diagnostic binary -- no add_test(), not added
# to the install() list below (mirrors test_optitrack_bridge.cpp/
# test_mpc_deadband.cpp precedent).
# ---------------------------------------------------------------------
add_executable(test_motor_calib_e2e
    tests/test_motor_calib_e2e.cpp
    src/MotorCalibCore.cpp
)
target_include_directories(test_motor_calib_e2e PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(test_motor_calib_e2e
        PRIVATE
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(test_motor_calib_e2e
        PRIVATE
            ${ZeroMQ_LIBRARIES}
    )
endif()

# ---------------------------------------------------------------------
# drive_test: short hardware drive-test tool -- drives a real robot a
# configurable distance (default 2m) straight and back through the real
# VESC driver, reusing CalibClient's ZmqDriverClient/DriverRestoreGuard/
# LiveChannels (the same machinery motor_calibration uses). Needs cppzmq +
# nlohmann (header-only, already on MPC_INCLUDE_DIRS) but NOT Boost/Eigen/
# Ceres/Qt -- mirrors motor_calibration's dependency-light footprint.
# ---------------------------------------------------------------------
add_executable(drive_test
    src/drive_test.cpp
    src/CalibClient.cpp
    src/MotorCalibCore.cpp
)
target_include_directories(drive_test PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(drive_test
        PRIVATE
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(drive_test
        PRIVATE
            ${ZeroMQ_LIBRARIES}
    )
endif()

# ---------------------------------------------------------------------
# test_drive_test: standalone, hardware-free end-to-end test for drive_test
# -- spawns real fake_vesc + vesc_driver child processes and plays OptiTrack
# itself (same scheme as test_motor_calib_e2e.cpp), then runs the real
# drive_test binary against them wrapped, asserting on its output/exit code.
# Test-only ZMQ ports 45950-45959 ONLY (see the file's own header comment).
# Standalone diagnostic binary -- no add_test(), not added to the install()
# list below (mirrors test_motor_calib_e2e.cpp precedent).
# ---------------------------------------------------------------------
add_executable(test_drive_test
    tests/test_drive_test.cpp
    src/MotorCalibCore.cpp
)
target_include_directories(test_drive_test PRIVATE ${MPC_INCLUDE_DIRS})

if(APPLE)
    target_link_libraries(test_drive_test
        PRIVATE
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(test_drive_test
        PRIVATE
            ${ZeroMQ_LIBRARIES}
    )
endif()

# ---------------------------------------------------------------------
# sustain_sweep: measures the SUSTAIN duty threshold -- the lowest duty that
# keeps an ALREADY-MOVING robot moving (as opposed to a prior breakaway
# sweep's threshold to get a STATIONARY robot moving at all). Reuses the
# SAME live-hardware machinery drive_test/motor_calibration already use
# (CalibClient's ZmqDriverClient/DriverRestoreGuard/LiveChannels,
# MotorCalibCore's LineFrame/VelocityEstimator) -- see src/sustain_sweep.cpp's
# own header comment for the full kick/ramp-down/drop-out procedure and
# safety-abort design. The pure kick/ramp/drop-out/abort-predicate logic
# lives in mpc::SustainSweepCore.h/.cpp (no sockets), unit-tested by
# sustain_sweep_tests below -- mirrors the MotorCalibCore/motor_calibration
# split precedent above. Needs cppzmq + nlohmann (header-only, already on
# MPC_INCLUDE_DIRS) but NOT Boost/Eigen/Ceres/Qt -- mirrors drive_test's
# dependency-light footprint.
# ---------------------------------------------------------------------
add_executable(sustain_sweep
    src/sustain_sweep.cpp
    src/SustainSweepCore.cpp
    src/CalibClient.cpp
    src/MotorCalibCore.cpp
)
target_include_directories(sustain_sweep PRIVATE ${MPC_INCLUDE_DIRS})
# Compiled-in default path to read governor.ff_gain/erpm_per_mps from for the
# final duty->speed conversion (see sustain_sweep.cpp's resolve_driver_
# constants()) -- mirrors motor_calibration's MOTOR_CALIB_DEFAULT_CONFIG_PATH
# precedent. Falls back to --ff-gain/--erpm-per-mps (or their compiled
# defaults) if this file is missing or doesn't parse -- never a hard error.
target_compile_definitions(sustain_sweep PRIVATE
    SUSTAIN_SWEEP_DEFAULT_DRIVER_CONFIG_PATH="${CMAKE_SOURCE_DIR}/VescDriver/config/driver_config.json"
)

if(APPLE)
    target_link_libraries(sustain_sweep
        PRIVATE
            ${ZMQ_LIB}
    )
else()
    target_link_libraries(sustain_sweep
        PRIVATE
            ${ZeroMQ_LIBRARIES}
    )
endif()
# sustain_sweep is a standalone, manually-run hardware diagnostic tool (like
# drive_test/motor_calibration) -- deliberately NOT added to the install()
# list below (mirrors drive_test's own precedent, not motor_calibration's).

# ---------------------------------------------------------------------
# sustain_sweep_tests: unit tests for mpc::SustainSweepCore's pure logic
# (duty ladder generation, drop-out/DropoutDetector debounce, the
# evaluate_sustain_abort() predicate, the sustain-duty<->speed conversion,
# compute_stats) -- no sockets, no spawned processes, mirrors motor_calib_
# tests.cpp/calibration_core_tests.cpp's harness style (plain bool
# test_xxx() functions registered in main(), no gtest dependency).
# Deliberately even lighter than motor_calib_tests: SustainSweepCore.h/.cpp
# needs no nlohmann/ZMQ/Boost/Eigen/Ceres -- plain C++ only.
# ---------------------------------------------------------------------
add_executable(sustain_sweep_tests
    tests/sustain_sweep_tests.cpp
    src/SustainSweepCore.cpp
)
target_include_directories(sustain_sweep_tests PRIVATE ${MPC_INCLUDE_DIRS})
# sustain_sweep_tests is a standalone, manually-run test binary (see this
# project's test-binary convention above at motor_calib_tests' own comment)
# -- no add_test(), not added to the install() list below.

# ---------------------------------------------------------------------
# calibration_core: UI-independent teleop-calibration data/fitting library
# (mpc::CalibrationCore -- see MPC/include/mpc/CalibrationCore.h's file
# header comment). Pure logic (no ZMQ sockets, no Qt -- every stream this
# needs is fed in by the caller via feed_pose()/feed_telemetry()/
# feed_command()), needs only nlohmann (header-only, already reachable via
# ${CMAKE_SOURCE_DIR}/include) -- mirrors MotorCalibCore.cpp's dependency-
# light footprint. Unlike MotorCalibCore.cpp (folded directly into each
# consumer's source list), this is a real STATIC LIBRARY target: a later
# Qt tab links it directly, so its include directory is exported PUBLIC
# (propagates to any target that links calibration_core, no separate
# target_include_directories needed on the consumer side).
# ---------------------------------------------------------------------
add_library(calibration_core STATIC
    src/CalibrationCore.cpp
)
target_include_directories(calibration_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
)

# ---------------------------------------------------------------------
# robot_calibration: headless CLI that fits calibration_core's velocity/
# steering maps from an already-recorded teleop session directory and
# exports the FROZEN OUTPUT SCHEMAS (velocity_calib.json/
# steering_angle_map.json) the robot-side driver loads. No live ZMQ feed
# here at all (unlike motor_calibration/drive_test) -- see
# src/robot_calibration.cpp's own header comment.
# ---------------------------------------------------------------------
add_executable(robot_calibration
    src/robot_calibration.cpp
)
target_include_directories(robot_calibration PRIVATE ${MPC_INCLUDE_DIRS})
target_link_libraries(robot_calibration PRIVATE calibration_core)

# ---------------------------------------------------------------------
# calibration_core_tests: standalone, manually-run test binary (see this
# project's test-binary convention above at motor_calib_tests' own comment)
# -- no add_test(), not added to the install() list below.
# ---------------------------------------------------------------------
add_executable(calibration_core_tests
    tests/calibration_core_tests.cpp
)
target_include_directories(calibration_core_tests PRIVATE ${MPC_INCLUDE_DIRS})
target_link_libraries(calibration_core_tests PRIVATE calibration_core)

install(TARGETS mpc_controller mpc_dummy_straight_mover mpc_robot_sim mpc_unit_tests
        optitrack_zmq_bridge fake_motive optitrack_unit_tests motor_calibration
        driver_telemetry_tests
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

find_package(Threads REQUIRED)
include(GNUInstallDirs)

set(PHASTAR_COMMON_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/ReloPush
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

set(PHASTAR_SHARED_RELOPUSH_SOURCES
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BinaryString.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/DubinsTools.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FromOMPL.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/PathPlanningTools.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/PrintInColor.cpp
)

function(phastar_apply_common_settings target_name)
    target_include_directories(${target_name} PRIVATE ${PHASTAR_COMMON_INCLUDE_DIRS})

    if(APPLE)
        target_link_libraries(${target_name}
            PRIVATE
                ${Boost_LIBRARIES}
                ${OMPL_LIBRARIES}
                ${Eigen_LIBRARIES}
                ${QT_WIDGET_TARGET}
                ${ODE_DIR}/lib/libode.dylib
        )
    else()
        target_link_libraries(${target_name}
            PRIVATE
                ${Boost_LIBRARIES}
                ${OMPL_LIBRARIES}
                ${Eigen_LIBRARIES}
                ${QT_WIDGET_TARGET}
                ${RELOPUSH_ABSL_LOG_LIBS}
        )
    endif()
endfunction()

function(phastar_link_zeromq target_name)
    if(APPLE)
        target_link_libraries(${target_name} PRIVATE ${ZMQ_LIB})
    else()
        target_link_libraries(${target_name} PRIVATE ${ZeroMQ_LIBRARIES})
    endif()
endfunction()

add_executable(phastar_demo
    src/PHAstar_demo.cpp
)
phastar_apply_common_settings(phastar_demo)

add_executable(phastar_push_demo
    src/PHAstar_push_demo.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    src/DqnAllocationSearch.cpp
    src/DqnFeaturesV2.cpp
    src/TransitionLogger.cpp
    src/GeometryExport.cpp
    src/PgTableExport.cpp
    src/EvalPlansCli.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(phastar_push_demo)
target_link_libraries(phastar_push_demo PRIVATE Threads::Threads)
phastar_link_zeromq(phastar_push_demo)

add_executable(phastar_push_replay_sequence
    src/PHAstar_push_replay_sequence.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    src/DqnAllocationSearch.cpp
    src/DqnFeaturesV2.cpp
    src/TransitionLogger.cpp
    src/GeometryExport.cpp
    src/PgTableExport.cpp
    src/EvalPlansCli.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(phastar_push_replay_sequence)
target_link_libraries(phastar_push_replay_sequence PRIVATE Threads::Threads)
phastar_link_zeromq(phastar_push_replay_sequence)

add_executable(phastar_unit_tests
    tests/phastar_unit_tests.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    src/DqnAllocationSearch.cpp
    src/DqnFeaturesV2.cpp
    src/TransitionLogger.cpp
    src/GeometryExport.cpp
    src/PgTableExport.cpp
    src/EvalPlansCli.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(phastar_unit_tests)
target_link_libraries(phastar_unit_tests PRIVATE Threads::Threads)
phastar_link_zeromq(phastar_unit_tests)
add_dependencies(phastar_unit_tests phastar_push_demo)

add_executable(test_executed_scenario_serialization
    ${CMAKE_SOURCE_DIR}/test/test_executed_scenario_serialization.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    src/DqnAllocationSearch.cpp
    src/DqnFeaturesV2.cpp
    src/TransitionLogger.cpp
    src/GeometryExport.cpp
    src/PgTableExport.cpp
    src/EvalPlansCli.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(test_executed_scenario_serialization)
target_link_libraries(test_executed_scenario_serialization PRIVATE Threads::Threads)
phastar_link_zeromq(test_executed_scenario_serialization)

add_executable(test_robot_controllers
    tests/test_robot_controllers.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(test_robot_controllers)
target_link_libraries(test_robot_controllers PRIVATE Threads::Threads)
phastar_link_zeromq(test_robot_controllers)

add_executable(test_mpc_full_loop
    tests/test_mpc_full_loop.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(test_mpc_full_loop)
target_link_libraries(test_mpc_full_loop PRIVATE Threads::Threads)
phastar_link_zeromq(test_mpc_full_loop)
add_dependencies(test_mpc_full_loop mpc_controller)

add_executable(test_sim_viz_handoff
    tests/test_sim_viz_handoff.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(test_sim_viz_handoff)
target_link_libraries(test_sim_viz_handoff PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_handoff)

# -----------------------------------------------------------------------
# Phase B: mars_sim_viz -- headless/GUI manager that owns execution once a
# MARS run hands a scored plan off to it (see MARS/include/SimVizHandoff.h
# for the MARS-side half of this protocol). SimVizCore.{h,cpp}
# (ControlServer/ScenarioModel/ExecutionManager/LocalizationListener) is
# display-free (Phase B1); SimVizWindow.{h,cpp} (Phase B2) is the QtWidgets
# GUI on top of it -- simviz_main.cpp defaults to the GUI, --headless keeps
# the window-free Phase B1 mode (what the automated tests below drive).
# Source list mirrors test_sim_viz_handoff's (same header dependency
# footprint: ExecutedScenarioSerialization.h + RobotTrajectoryBuilder.h).
add_executable(mars_sim_viz
    src/simviz/SimVizCore.cpp
    src/simviz/SimVizWindow.cpp
    src/simviz/simviz_main.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(mars_sim_viz PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(mars_sim_viz)
target_link_libraries(mars_sim_viz PRIVATE Threads::Threads)
phastar_link_zeromq(mars_sim_viz)

# Phase B unit tests (headless/offscreen -- see MARS/tests/*.cpp headers for
# what each covers).
add_executable(test_sim_viz_control
    tests/test_sim_viz_control.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(test_sim_viz_control)
target_link_libraries(test_sim_viz_control PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_control)

# ISSUE 1 regression test (see test_sim_viz_pose_retention.cpp's header
# comment): fake sleeping child binaries (same as test_sim_viz_control,
# above) plus a hand-rolled ZMQ PUB standing in for mpc_robot_sim's
# localization publish, asserting ExecutionManager::localization()'s
# latest-pose cache survives ERR/ABORT teardown and resets on a fresh
# EXECUTE. In-process SimVizManager, no separate mars_sim_viz spawn needed.
add_executable(test_sim_viz_pose_retention
    tests/test_sim_viz_pose_retention.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_pose_retention PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(test_sim_viz_pose_retention)
target_link_libraries(test_sim_viz_pose_retention PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_pose_retention)

add_executable(test_sim_viz_scenario_model
    tests/test_sim_viz_scenario_model.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_scenario_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
target_compile_definitions(test_sim_viz_scenario_model PRIVATE
    MARS_TEST_FIXTURE_SCN="${CMAKE_SOURCE_DIR}/results/fix1_gates/boss13_repro.scn.b64"
)
phastar_apply_common_settings(test_sim_viz_scenario_model)
target_link_libraries(test_sim_viz_scenario_model PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_scenario_model)

# Phase B2 offscreen render smoke test: loads a real .scn.b64 fixture
# directly into a SimVizCanvas (no ExecutionManager/network/process spawn
# involved -- see test_sim_viz_window_render.cpp's header comment) and
# grabs it to a QImage under QT_QPA_PLATFORM=offscreen. Deterministic and
# fast like test_sim_viz_scenario_model, so it's wired into ctest too.
add_executable(test_sim_viz_window_render
    tests/test_sim_viz_window_render.cpp
    src/simviz/SimVizCore.cpp
    src/simviz/SimVizWindow.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_window_render PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
target_compile_definitions(test_sim_viz_window_render PRIVATE
    MARS_TEST_FIXTURE_SCN="${CMAKE_SOURCE_DIR}/results/fix1_gates/boss13_repro.scn.b64"
    MARS_TEST_RENDER_OUT_DIR="${CMAKE_CURRENT_BINARY_DIR}"
)
phastar_apply_common_settings(test_sim_viz_window_render)
target_link_libraries(test_sim_viz_window_render PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_window_render)

# RIGHT-SIDE MONITOR PANEL (core layer): LocalizationListener's new
# telemetry tracking (SimVizCore.h/.cpp) -- modeled directly on
# test_sim_viz_pose_retention.cpp immediately above (same fake-sleeping-
# child-binary + hand-rolled ZMQ PUB technique, same TEST PORT BLOCK,
# in-process SimVizManager, no separate mars_sim_viz spawn needed).
# Deterministic and fast, so it's wired into ctest too.
add_executable(test_sim_viz_telemetry
    tests/test_sim_viz_telemetry.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_telemetry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(test_sim_viz_telemetry)
target_link_libraries(test_sim_viz_telemetry PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_telemetry)

# RIGHT-SIDE MONITOR PANEL (widget layer) + MOTOR STALL UI controls:
# offscreen smoke test for SimVizWindow's new per-robot monitor cells and
# "Motor stall" checkbox/spinbox -- same style/technique as
# test_sim_viz_window_render.cpp immediately above (a REAL SimVizWindow
# driving a REAL ExecutionManager against fake sleeping-shell children, no
# real mpc_controller/mpc_robot_sim spawn). Deterministic and fast, so it's
# wired into ctest too.
add_executable(test_sim_viz_monitor_panel
    tests/test_sim_viz_monitor_panel.cpp
    src/simviz/SimVizCore.cpp
    src/simviz/SimVizWindow.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_monitor_panel PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(test_sim_viz_monitor_panel)
target_link_libraries(test_sim_viz_monitor_panel PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_monitor_panel)

# Standalone headless integration test (spawns real mars_sim_viz --headless
# + real mpc_controller/mpc_robot_sim children against a synthetic short
# scenario) -- mirrors MPCController/tests/test_mpc_sim_loop.cpp's style:
# a standalone diagnostic binary, not necessarily wired into ctest.
add_executable(test_sim_viz_integration
    tests/test_sim_viz_integration.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(test_sim_viz_integration)
target_link_libraries(test_sim_viz_integration PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_integration)
add_dependencies(test_sim_viz_integration mars_sim_viz mpc_controller mpc_robot_sim)

# --execute / --exit-on-done direct-load test (see MARS/tests/
# test_sim_viz_execute.cpp's header comment for the full breakdown). Same
# standalone-binary style as test_sim_viz_integration (spawns the real
# mars_sim_viz --headless binary), plus SimVizCore.cpp linked directly so
# Part D can call ScenarioModel::load_from_file() in-process against the
# real results/sim_handoff/greedy.scn.b64 output without spawning a process
# for that check.
add_executable(test_sim_viz_execute
    tests/test_sim_viz_execute.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_execute PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
target_compile_definitions(test_sim_viz_execute PRIVATE
    MARS_TEST_GREEDY_HANDOFF_SCN="${CMAKE_SOURCE_DIR}/results/sim_handoff/greedy.scn.b64"
    MARS_TEST_FIXTURE_SCN="${CMAKE_SOURCE_DIR}/results/fix1_gates/boss13_repro.scn.b64"
)
phastar_apply_common_settings(test_sim_viz_execute)
target_link_libraries(test_sim_viz_execute PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_execute)
add_dependencies(test_sim_viz_execute mars_sim_viz mpc_controller mpc_robot_sim)

# FEATURE 2C: standalone pause/resume/restart test -- same style as
# test_sim_viz_execute (spawns the real mars_sim_viz --headless binary +
# real mpc_controller/mpc_robot_sim children against a synthetic scenario),
# not wired into ctest.
add_executable(test_sim_viz_pause
    tests/test_sim_viz_pause.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_pause PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(test_sim_viz_pause)
target_link_libraries(test_sim_viz_pause PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_pause)
add_dependencies(test_sim_viz_pause mars_sim_viz mpc_controller mpc_robot_sim)

# FEATURE C: standalone motor-noise-slider test (spawns the real
# mars_sim_viz --headless binary + real mpc_controller/mpc_robot_sim
# children), same style as test_sim_viz_pause immediately above -- not wired
# into ctest.
add_executable(test_sim_viz_noise
    tests/test_sim_viz_noise.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_noise PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(test_sim_viz_noise)
target_link_libraries(test_sim_viz_noise PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_noise)
add_dependencies(test_sim_viz_noise mars_sim_viz mpc_controller mpc_robot_sim)

# MOTOR STALL: standalone motor-stall (min-speed deadband UI) plumbing test
# (spawns the real mars_sim_viz --headless binary + real mpc_controller/
# mpc_robot_sim children), same style as test_sim_viz_noise immediately
# above -- not wired into ctest.
add_executable(test_sim_viz_stall
    tests/test_sim_viz_stall.cpp
    src/simviz/SimVizCore.cpp
    src/CsvLogging.cpp
    src/ResultVisualization.cpp
    src/SafeParking.cpp
    src/PlanningHelpers.cpp
    src/CollisionScheduling.cpp
    src/ReloPushSetup.cpp
    src/AllocationSearch.cpp
    src/TaskExecution.cpp
    src/RuntimeOptionsParsing.cpp
    src/DataLoading.cpp
    src/SearchOrchestrator.cpp
    src/SimVizHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${CMAKE_SOURCE_DIR}/src/ReloPush/BatchInstanceParcer.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
target_include_directories(test_sim_viz_stall PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/simviz
)
phastar_apply_common_settings(test_sim_viz_stall)
target_link_libraries(test_sim_viz_stall PRIVATE Threads::Threads)
phastar_link_zeromq(test_sim_viz_stall)
add_dependencies(test_sim_viz_stall mars_sim_viz mpc_controller mpc_robot_sim)

if(BUILD_TESTING)
    add_test(NAME phastar_unit_tests COMMAND phastar_unit_tests)
    add_test(NAME test_robot_controllers COMMAND test_robot_controllers)
    add_test(NAME test_mpc_full_loop COMMAND test_mpc_full_loop)
    add_test(NAME test_sim_viz_handoff COMMAND test_sim_viz_handoff)
    add_test(NAME test_sim_viz_control COMMAND test_sim_viz_control)
    add_test(NAME test_sim_viz_pose_retention COMMAND test_sim_viz_pose_retention)
    add_test(NAME test_sim_viz_scenario_model COMMAND test_sim_viz_scenario_model)
    add_test(NAME test_sim_viz_window_render COMMAND test_sim_viz_window_render)
    add_test(NAME test_sim_viz_telemetry COMMAND test_sim_viz_telemetry)
    add_test(NAME test_sim_viz_monitor_panel COMMAND test_sim_viz_monitor_panel)
ENDIF()

INSTALL(TARGETS
    phastar_demo
    phastar_push_demo
    phastar_push_replay_sequence
    phastar_unit_tests
    test_robot_controllers
    test_mpc_full_loop
    test_sim_viz_handoff
    mars_sim_viz
    test_sim_viz_control
    test_sim_viz_scenario_model
    test_sim_viz_window_render
    test_sim_viz_monitor_panel
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

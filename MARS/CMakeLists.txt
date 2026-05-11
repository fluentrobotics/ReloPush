find_package(Threads REQUIRED)
include(GNUInstallDirs)

set(PHASTAR_COMMON_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fromReloPush
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/ReloPush
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

set(PHASTAR_SHARED_RELOPUSH_SOURCES
    ${CMAKE_SOURCE_DIR}/src/ReloPush/base64.cpp
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
                absl::log_initialize
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
    ${CMAKE_SOURCE_DIR}/src/ReloPush/FinalSequenceHandoff.cpp
    ${PHASTAR_SHARED_RELOPUSH_SOURCES}
)
phastar_apply_common_settings(phastar_push_replay_sequence)
target_link_libraries(phastar_push_replay_sequence PRIVATE Threads::Threads)
phastar_link_zeromq(phastar_push_replay_sequence)

add_executable(phastar_unit_tests
    tests/phastar_unit_tests.cpp
)
phastar_apply_common_settings(phastar_unit_tests)
add_dependencies(phastar_unit_tests phastar_push_demo)

if(BUILD_TESTING)
    add_test(NAME phastar_unit_tests COMMAND phastar_unit_tests)
endif()

install(TARGETS
    phastar_demo
    phastar_push_demo
    phastar_push_replay_sequence
    phastar_unit_tests
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

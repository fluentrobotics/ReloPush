cmake_minimum_required(VERSION 3.16)
project(mars_launcher LANGUAGES CXX)

# Standalone CMake project for the MARS/ReloPush Qt Widgets launcher.
# This is intentionally NOT included from the parent repo CMakeLists.txt;
# configure/build it directly:
#   cmake -S ui -B ui/build -DCMAKE_BUILD_TYPE=Release
#   cmake --build ui/build -j4

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

# Prefer Qt6, fall back to Qt5 (mirrors the parent CMakeLists.txt strategy).
find_package(Qt6 COMPONENTS Widgets QUIET)
if (Qt6_FOUND)
    set(QT_TARGET_PREFIX Qt6)
else()
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    set(QT_TARGET_PREFIX Qt5)
endif()

add_executable(mars_launcher
    main.cpp
    LauncherWindow.cpp
    LauncherWindow.h
)

target_link_libraries(mars_launcher PRIVATE ${QT_TARGET_PREFIX}::Widgets)

# Bake in the default location of the option catalog so the launcher can find
# it without relying on the current working directory.
target_compile_definitions(mars_launcher PRIVATE
    DEFAULT_SPEC_PATH="${CMAKE_CURRENT_SOURCE_DIR}/options_spec.json"
    DEFAULT_BASIC_REFINEMENTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/basic_refinements.json"
)

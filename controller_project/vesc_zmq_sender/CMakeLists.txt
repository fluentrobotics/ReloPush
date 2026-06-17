cmake_minimum_required(VERSION 3.16)

project(vesc_zmq_sender LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
  cppzmq
  GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
  GIT_TAG v4.10.0
)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)

FetchContent_MakeAvailable(cppzmq nlohmann_json)

find_package(Threads REQUIRED)

find_package(ZeroMQ QUIET)
if (ZeroMQ_FOUND)
  if (TARGET ZeroMQ::ZeroMQ)
    set(ZMQ_LIB ZeroMQ::ZeroMQ)
  elseif (TARGET libzmq)
    set(ZMQ_LIB libzmq)
  else()
    set(ZMQ_LIB zmq)
  endif()
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBZMQ REQUIRED libzmq)
  add_library(libzmq INTERFACE)
  target_include_directories(libzmq INTERFACE ${LIBZMQ_INCLUDE_DIRS})
  target_link_directories(libzmq INTERFACE ${LIBZMQ_LIBRARY_DIRS})
  target_link_libraries(libzmq INTERFACE ${LIBZMQ_LIBRARIES})
  set(ZMQ_LIB libzmq)
endif()

add_library(base64
  third_party/base64/base64.cpp
)
target_include_directories(base64 PUBLIC third_party/base64)

set(CPPZMQ_LIB cppzmq)

add_executable(vesc_zmq_sender
  src/main.cpp
)

target_link_libraries(vesc_zmq_sender
  PRIVATE
    base64
    nlohmann_json::nlohmann_json
    ${CPPZMQ_LIB}
    ${ZMQ_LIB}
    Threads::Threads
)

# Build only ArduPilotPlugin from an ardupilot_gazebo checkout, against Gazebo Harmonic.
# The upstream project also builds camera/GStreamer plugins that this simulation does
# not use and that are awkward to build on macOS.
#
#   export ARDUPILOT_GAZEBO_DIR=$HOME/ardupilot_gazebo
#   cmake -S crazyflie_scripts/gazebo_swarm/plugin -B build/gz_plugin -DCMAKE_BUILD_TYPE=RelWithDebInfo
#   cmake --build build/gz_plugin -j
#
# Build with the same Gazebo installation that runs `gz sim`.
cmake_minimum_required(VERSION 3.16)
project(crazyflie_ardupilot_gazebo_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ARDUPILOT_GAZEBO_SOURCE_DIR "$ENV{ARDUPILOT_GAZEBO_DIR}" CACHE PATH
    "Path to an ArduPilot/ardupilot_gazebo checkout")
if(NOT EXISTS "${ARDUPILOT_GAZEBO_SOURCE_DIR}/src/ArduPilotPlugin.cc")
  message(FATAL_ERROR
    "ardupilot_gazebo sources not found at '${ARDUPILOT_GAZEBO_SOURCE_DIR}'. "
    "Set ARDUPILOT_GAZEBO_DIR or -DARDUPILOT_GAZEBO_SOURCE_DIR.")
endif()

find_package(gz-sim8 REQUIRED)

add_library(ArduPilotPlugin SHARED
  "${ARDUPILOT_GAZEBO_SOURCE_DIR}/src/ArduPilotPlugin.cc"
  "${ARDUPILOT_GAZEBO_SOURCE_DIR}/src/SocketUDP.cc"
  "${ARDUPILOT_GAZEBO_SOURCE_DIR}/src/Util.cc")
target_include_directories(ArduPilotPlugin PRIVATE "${ARDUPILOT_GAZEBO_SOURCE_DIR}/include")
target_link_libraries(ArduPilotPlugin PRIVATE gz-sim8::gz-sim8)

include_directories(${CMAKE_SOURCE_DIR}/swarmspaces)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(SwarmSpaces MODULE SwarmSpaces.h SwarmSpaces.cpp)
target_link_libraries(SwarmSpaces
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  SwarmTuple)

add_library(SwarmTuple MODULE swarmtuple.cpp)
target_link_libraries(SwarmTuple
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
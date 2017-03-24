include_directories(${CMAKE_SOURCE_DIR}/controllers)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

link_directories(${CMAKE_SOURCE_DIR}/swarmspaces)

add_library(ck_footbot_diffusion SHARED ck_footbot_diffusion.h ck_footbot_diffusion.cpp)
target_link_libraries(ck_footbot_diffusion
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  SwarmSpaces
  SwarmTuple)

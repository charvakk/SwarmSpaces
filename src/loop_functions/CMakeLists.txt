include_directories(${CMAKE_SOURCE_DIR}/loop_functions)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

link_directories(${CMAKE_SOURCE_DIR}/swarmspaces)
link_directories(${CMAKE_SOURCE_DIR}/controllers)

add_library(loop_functions MODULE swarmspaces_loop_functions.h swarmspaces_loop_functions.cpp)
target_link_libraries(loop_functions
  argos3core_simulator
  argos3plugin_simulator_footbot
  )

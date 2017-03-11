# SwarmSpaces
Pre-requisites:
1. Download and install the ARGoS simulator from http://www.argos-sim.info/download.php

Compiling and running:
1. Make a 'build' directory outside the 'src' folder and build the project using cmake.
  - mkdir build
  - cd build
  - cmake -DCMAKE_BUILD_TYPE=Debug (or Release) ..
  - make
2. Run "argos3 -c src/experiments/ck_diffusion.argos" from outside the src directory

#!/bin/bash

# Super4PCS
# See: http://geometry.cs.ucl.ac.uk/projects/2014/super4PCS/

git clone https://github.com/nmellado/Super4PCS.git
cd Super4PCS/
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo make install

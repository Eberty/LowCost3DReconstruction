#!/bin/bash

# Set current directory
CUR_DIR=$PWD

# ------------- OpenMVS -------------

#Prepare and empty machine for building:
sudo apt update
sudo apt install -y build-essential git mercurial cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev libxmu-dev libxi-dev
main_path=`pwd`

#Eigen (Required)
hg clone https://bitbucket.org/eigen/eigen#3.3
cd eigen
mkdir build
cd build
cmake ..
make -j$(nproc) && sudo make install

cd $CUR_DIR

# Boost (Required)
sudo apt install -y libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

# OpenCV (Required)
sudo apt install -y libopencv-dev

# CGAL (Required)
sudo apt install -y libcgal-dev libcgal-qt5-dev

# VCGLib (Required)
git clone https://github.com/cdcseacave/VCG.git vcglib

# Ceres (Required)
sudo apt install -y libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
cd ceres-solver
# git checkout $(git describe --tags) # Checkout the latest release
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc) && sudo make install

cd $CUR_DIR

# GLFW3 (Optional)
sudo apt install -y freeglut3-dev libglew-dev libglfw3-dev

# OpenMVS
git clone https://github.com/cdcseacave/openMVS.git openMVS
cd openMVS
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT="$main_path/vcglib" -DOpenMVS_USE_CUDA=ON

# If you want to use OpenMVS as shared library, add to the CMake command:
# -DBUILD_SHARED_LIBS=ON

# Install OpenMVS library
make -j$(nproc) && sudo make install

# OpenMVS bin dir: /usr/local/bin/OpenMVS

cd $CUR_DIR

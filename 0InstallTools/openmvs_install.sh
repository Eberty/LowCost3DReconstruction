#!/bin/bash

# Ubuntu 16.04 is used next as the example linux distribution.

# Set current directory
CUR_DIR=$PWD

# ------------- OpenMVS -------------

#Prepare and empty machine for building:
sudo apt update
sudo apt -y install build-essential git mercurial cmake libpng-dev libjpeg-dev libtiff-dev libglu1-mesa-dev libxmu-dev libxi-dev
main_path=`pwd`

#Eigen (Required)
hg clone https://bitbucket.org/eigen/eigen#3.2
mkdir eigen_build && cd eigen_build
cmake . ../eigen
make -j2 && sudo make install
cd ..

# Boost (Required)
sudo apt -y install libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

# OpenCV (Required)
sudo apt -y install libopencv-dev

# CGAL (Required)
sudo apt -y install libcgal-dev libcgal-qt5-dev

# VCGLib (Required)
git clone https://github.com/cdcseacave/VCG.git vcglib

# Ceres (Required)
sudo apt -y install libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
mkdir ceres_build && cd ceres_build
cmake . ../ceres-solver/ -DMINIGLOG=ON -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j2 && sudo make install
cd ..

# GLFW3 (Optional)
sudo apt -y install freeglut3-dev libglew-dev libglfw3-dev

# OpenMVS
git clone https://github.com/cdcseacave/openMVS.git openMVS
mkdir openMVS_build && cd openMVS_build
cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT="$main_path/vcglib"

# CUDA OFF
# cmake . ../openMVS -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT="$main_path/vcglib" -DOpenMVS_USE_CUDA=OFF

# If you want to use OpenMVS as shared library, add to the CMake command:
# -DBUILD_SHARED_LIBS=ON

# Install OpenMVS library (optional):
make -j2 && sudo make install

# OpenMVS bin dir: /usr/local/bin/OpenMVS

cd $CUR_DIR

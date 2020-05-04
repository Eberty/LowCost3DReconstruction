#!/bin/bash

# In newer distributions you can just run:
# sudo apt install -y colmap

# Set current directory
CUR_DIR=$PWD

# ------------- COLMAP -------------
# Recommended dependencies: CUDA (at least version 7.X)

# Dependencies from the default Ubuntu repositories:
sudo apt update
sudo apt install -y \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev

# Under Ubuntu 16.04/18.04 the CMake configuration scripts of CGAL are broken and you must also install the CGAL Qt5 package:
sudo apt install libcgal-qt5-dev -y

# Install Ceres Solver:
sudo apt install -y libatlas-base-dev libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver ceres-solver
cd ceres-solver
# git checkout $(git describe --tags) # Checkout the latest release
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
make -j$(nproc) && sudo make install

cd $CUR_DIR

# Configure and compile COLMAP:
git clone https://github.com/colmap/colmap.git colmap
cd colmap
git checkout dev
mkdir build
cd build
cmake ..

# Under newer Ubuntu versions it might be necessary to explicitly select the used GCC version due to compatiblity issues with CUDA, which can be done as:
# sudo apt install gcc-6 g++-6
# CC=/usr/bin/gcc-6 CXX=/usr/bin/g++-6 cmake ..

make -j$(nproc) && sudo make install

# Run COLMAP:
# colmap -h
# colmap gui

cd $CUR_DIR

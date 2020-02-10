#!/bin/bash

# Tools
sudo apt update
sudo apt install -y wget tar

# Set current directory
CUR_DIR=$PWD

# Clone latest PCL
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
tar -vzxf pcl-1.9.1.tar.gz
rm pcl-1.9.1.tar.gz
mv pcl-pcl-1.9.1 pcl
cd pcl

# Install prerequisites
sudo apt install -y g++ \
    cmake cmake-gui \
    doxygen \
    mpi-default-dev openmpi-bin openmpi-common \
    libflann1.8 libflann-dev \
    libeigen3-dev \
    libboost-all-dev \
    libvtk6-dev libvtk6.2 libvtk6.2-qt \
    'libqhull*' \
    libusb-dev \
    libgtest-dev \
    git-core freeglut3-dev pkg-config \
    build-essential libxmu-dev libxi-dev \
    libusb-1.0-0-dev graphviz mono-complete \
    qt-sdk openjdk-9-jdk openjdk-9-jre \
    phonon-backend-gstreamer \
    phonon-backend-vlc \
    libopenni-dev libopenni2-dev

# Compile and install PCL
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_apps=ON -DBUILD_examples=OFF -DBUILD_GPU=OFF ..
make -j$(nproc) && sudo make install

cd $CUR_DIR


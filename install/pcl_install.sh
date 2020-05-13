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
sudo apt install -y g++ cmake cmake-gui doxygen git freeglut3-dev pkg-config build-essential libusb-1.0-0-dev graphviz mono-complete
sudo apt install -y mpi-default-dev openmpi-bin openmpi-common
sudo apt install -y libflann1.8
sudo apt install -y libflann-dev
sudo apt install -y libeigen3-dev libboost-all-dev
sudo apt install -y libvtk6-dev libvtk6.2 libvtk6.2-qt
sudo apt install -y libvtk6-dev libvtk6.3 libvtk6.3-qt
sudo apt install -y 'libqhull*'
sudo apt install -y libusb-dev
sudo apt install -y libgtest-dev
sudo apt install -y libxmu-dev libxi-dev
sudo apt install -y qt-sdk
sudo apt install -y openjdk-9-jdk openjdk-9-jre
sudo apt install -y phonon-backend-gstreamer phonon-backend-vlc
sudo apt install -y libopenni-dev libopenni2-dev

# Compile and install PCL
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_apps=ON -DBUILD_examples=OFF -DBUILD_GPU=OFF ..
make -j$(nproc) && sudo make install

cd $CUR_DIR


#!/bin/bash

# Requirements: USB 3.0 controller. USB 2 is not supported.

# Set current directory
CUR_DIR=$PWD

# ------------- libfreenect2 -------------

sudo apt update

# Install git:
sudo apt install -y git

# Install build tools:
sudo apt install -y build-essential cmake pkg-config

# Install libusb
sudo apt install -y libusb-1.0-0-dev

# Install TurboJPEG
# (Ubuntu 14.04 to 16.04)
sudo apt install -y libturbojpeg libjpeg-turbo8-dev
# (Debian/Ubuntu 17.10 and newer)
sudo apt install -y libturbojpeg0-dev

# Install OpenGL
sudo apt install -y libglfw3-dev

# Install OpenCL (optional) 
sudo apt install -y beignet-dev
sudo apt install -y opencl-headers

# Install CUDA (optional, Nvidia only)
# Download CUDA Toolkit and install it. You MUST install the samples too

# Install VAAPI (optional, Intel only) 
sudo apt install -y libva-dev libjpeg-dev

# Install OpenNI2 (optional) 
sudo apt install -y libopenni2-dev

# Download libfreenect2 source 
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build
cd build
cmake ..
make -j$(nproc) && sudo make install

# cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
# make -j$(nproc) && make install

# You need to specify the CMake based third-party application to find libfreenect2:
# cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect:

sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/

# Run the test program: 
# ./bin/Protonect

# Run OpenNI2 test (optional):
# sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2

cd $CUR_DIR

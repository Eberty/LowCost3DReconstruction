#!/bin/bash

CUR_DIR=${PWD}

# Tools
sudo apt update
sudo apt install \
	build-essential \
	cmake \
	pkg-config \
	meshlab \
	gthumb \
	ffmpeg \
	libopencv-dev \
	libfreenect-dev \
	libfreenect0.5 \
	libomp-dev \
	libboost-all-dev \
	libsoil-dev \
	freeglut3-dev \
	libglew-dev \
	libxxf86vm-dev -y

sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.a /usr/lib/libglut.a
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so /usr/lib/libglut.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so.3 /usr/lib/libglut.so.3
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so.3.9.0 /usr/lib/libglut.so.3.9.0

# Compile and install Libfreenect
sudo apt-get install git cmake build-essential libusb-1.0-0-dev
sudo apt-get install freeglut3-dev libxmu-dev libxi-dev # only if you are building the examples
git clone https://github.com/OpenKinect/libfreenect

cd libfreenect
sudo cp ./platform/linux/udev/51-kinect.rules /etc/udev/rules.d/
mkdir build
cd build
cmake -L .. # -L lists all the project options
make -j$(nproc) && sudo make install

cd ${CUR_DIR}

# Compile and install Meshlab
cd ~
git clone --recursive https://github.com/cnr-isti-vclab/meshlab
bash meshlab/install/linux/linux_setup_env_ubuntu.sh
bash meshlab/install/linux/linux_build.sh

cd ${CUR_DIR}

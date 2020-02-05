#!/bin/bash

# Set current directory
CUR_DIR=$PWD

# ------------- TEASER-plusplus -------------

sudo apt update

cmake_version="$(echo $(cmake --version) | grep -Po '(\d+.*\d+)(?=\.)')"
if [[ ( ${cmake_version%%.*} < 3 ) || ( ${cmake_version%%.*} == 3 && ( 1 -eq "$(echo "${cmake_version##*.} < 10" | bc)" ) ) ]]; then
    echo "Cmake minimum version required: 3.10"

	# Install Cmake latest version:
	# See: https://anglehit.com/how-to-install-the-latest-version-of-cmake-via-command-line/
	sudo apt purge cmake
	version=3.10
	build=3
	wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
	tar -xzvf cmake-$version.$build.tar.gz
	rm cmake-$version.$build.tar.gz
	cd cmake-$version.$build/
	./bootstrap
	make -j$(nproc) && sudo make install
	cmake --version
	cd $CUR_DIR
fi

#Eigen (Required)
hg clone https://bitbucket.org/eigen/eigen#3.3
cd eigen
mkdir build
cd build
cmake ..
make -j$(nproc) && sudo make install

cd $CUR_DIR

# PCL (optional)
sudo apt install -y pcl-tools

# Boost (optional)
sudo apt install -y libboost-iostreams-dev libboost-program-options-dev libboost-system-dev libboost-serialization-dev

# Configure and compile TEASER-plusplus:
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus
mkdir build
cd build
cmake ..
make -j$(nproc) && sudo make install

cd $CUR_DIR

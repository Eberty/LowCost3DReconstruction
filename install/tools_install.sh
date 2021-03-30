#!/bin/bash

# Set current directory
CUR_DIR=${PWD}

# Dependencies from the default Ubuntu repositories
sudo apt update
sudo apt install -y \
  bc \
  beignet-dev \
  build-essential \
  cmake \
  curl \
  ffmpeg \
  freeglut3-dev \
  fuse \
  git \
  gthumb \
  libatlas-base-dev \
  libboost-all-dev \
  libceres-dev \
  libcgal-dev \
  libcgal-qt5-dev \
  libeigen3-dev \
  libfreeimage-dev \
  libfreenect-dev \
  libfreenect0.5 \
  libgflags-dev \
  libglew-dev \
  libglfw3-dev \
  libglu1-mesa-dev \
  libgoogle-glog-dev \
  libjpeg-dev \
  libomp-dev \
  libopencv-dev \
  libopenni2-dev \
  libpcl-dev \
  libpng-dev \
  libqt5opengl5-dev \
  libsoil-dev \
  libsuitesparse-dev \
  libtiff-dev \
  libturbojpeg \
  libusb-1.0-0-dev \
  libva-dev \
  libxi-dev \
  libxmu-dev \
  libxxf86vm-dev \
  mercurial \
  meshlab \
  opencl-headers \
  pkg-config \
  qtbase5-dev \
  tar \
  wget

sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.a /usr/lib/libglut.a
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so /usr/lib/libglut.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so.3 /usr/lib/libglut.so.3
sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so.3.9.0 /usr/lib/libglut.so.3.9.0

# ------------- git-lfs -------------

curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install -y git-lfs
git lfs install

# ------------- cmake -------------

cmake_version="$(echo $(cmake --version) | grep -Po '(\d+.*\d+)(?=\.)')"
if [[ ( ${cmake_version%%.*} < 3 ) || ( ${cmake_version%%.*} == 3 && ( 1 -eq "$(echo "${cmake_version##*.} < 10" | bc)" ) ) ]]; then
  echo "Cmake minimum required version: 3.10"

  sudo apt purge cmake -y
  version=3.10
  build=3
  wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
  tar -xzvf cmake-$version.$build.tar.gz > /dev/null
  rm cmake-$version.$build.tar.gz
  cd cmake-$version.$build

  ./bootstrap
  make -j$(nproc) && sudo make install

  hash -r
  cd ${CUR_DIR}
fi

sudo ldconfig

# ------------- libfreenect -------------

git clone https://github.com/OpenKinect/libfreenect.git libfreenect
cd libfreenect
sudo cp ./platform/linux/udev/51-kinect.rules /etc/udev/rules.d/
mkdir build
cd build
cmake -L ..
make -j$(nproc) && sudo make install

cd ${CUR_DIR}

# ------------- libfreenect2 -------------

# Requirements: USB 3.0 controller. USB 2 is not supported

sudo apt install -y libjpeg-turbo8-dev # Ubuntu 14.04 to 16.04
sudo apt install -y libturbojpeg0-dev # Ubuntu 17.10 and newer

git clone https://github.com/OpenKinect/libfreenect2.git libfreenect2
cd libfreenect2
sudo cp ./platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
mkdir build
cd build
cmake ..
make -j$(nproc) && sudo make install

cd ${CUR_DIR}

# ------------- Colmap -------------

git clone https://github.com/colmap/colmap.git colmap
cd colmap
mkdir build
cd build
cmake ..
make -j$(nproc) && sudo make install

# Under newer Ubuntu versions it might be necessary to explicitly select the used GCC version due to compatiblity issues with CUDA
# sudo apt install -y gcc-6 g++-6
# CC=/usr/bin/gcc-6 CXX=/usr/bin/g++-6 cmake ..

cd ${CUR_DIR}

# ------------- OpenMVS -------------

git clone https://github.com/cdcseacave/VCG.git vcglib # VCGLib (Required)

git clone https://github.com/cdcseacave/openMVS.git openMVS
cd openMVS
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DVCG_ROOT="${CUR_DIR}/vcglib" -DOpenMVS_USE_CUDA=ON
make -j$(nproc) && sudo make install

cd ${CUR_DIR}

# ------------- Super4PCS -------------

git clone https://github.com/nmellado/Super4PCS.git Super4PCS
cd Super4PCS
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo make install

cd ${CUR_DIR}

# ------------- LowCost3DReconstruction -------------

git clone https://github.com/Eberty/LowCost3DReconstruction.git LowCost3DReconstruction
cd LowCost3DReconstruction
mkdir build
cd build
cmake ..
make -j$(nproc) && sudo make install

cd ${CUR_DIR}
